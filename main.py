#import LoRaDuplexCallback
#import LoRaPingPong
#import LoRaSender
#import LoRaReceiver
import gc, os
import _thread
from machine import SPI, UART, Pin
import network
from time import sleep
import config_lora
from sx127x import SX127x
from controller_esp32 import ESP32Controller
from uPySensors.mpu9250 import MPU9250
from uPySensors.ssd1306_i2c import Display
from uPySensors.ublox_gps import MicropyGPS
from uPySensors.sdcard import SDCard

# settings
_booting = 0
_sleeping = 1
_moving  = 2

settings = {}
settings['irq'] = 0
settings['status'] = _booting

def collect_garbage():
    gc.collect()
    print('[Memory - free: {}   allocated: {}]'.format(gc.mem_free(), gc.mem_alloc()))

# pretty networks
def netpretty(network):
    net = {'name': network[0], 'bssid': str(prettify(network[1])), 'rssi': network[3], 'channel': network[2]}
    return net

# prettify mac address
def prettify(mac_binary):
    return ':'.join('%02x' % (b) for b in mac_binary)

# imu handler
moving_text = '.......'
moving_limit = 6
def mpu_irq_handler(handler):
    global settings
    settings['irq'] = settings['irq'] + 1
    if(settings['irq'] > moving_limit):
        settings['status'] = 2
        #display.poweron()
        display.show_text_wrap("Booting...")
        sensor.disable_irq_mode()
    else:
        display.show_text_wrap(moving_text[0:settings['irq']])

# communication
msgCount = 0                           # count of outgoing messages
intervalUpdates = [  1000,       # imu
                    10000,       # gps
                    20000,       # wlan
                     1000,       # display
                     5000,       # save data
                    10000]       # send data lora

controller = ESP32Controller()
lora = controller.add_transceiver(SX127x(name = 'LoRa'),
                                  pin_id_ss = ESP32Controller.PIN_ID_FOR_LORA_SS,
                                  pin_id_RxDone = ESP32Controller.PIN_ID_FOR_LORA_DIO0)
def gen_message(NODE_NAME, message, millisecond):
    return "{} {} {}".format(NODE_NAME, message, millisecond)

def on_receive(lora, payload):
    print('lora handler')

lora.onReceive(on_receive)

# imu settings
sensor = MPU9250(-1)
sensor.accel_range = 0
sensor.gyro_range = 1
sensor.filter_range = 1
irq_pin = Pin(25, mode=Pin.IN)
irq_pin.irq(trigger=Pin.IRQ_RISING, handler=mpu_irq_handler)

# display
display = Display()

# sd cards
spi_sd = SPI(1, sck = Pin(23, Pin.OUT), mosi = Pin(13, Pin.OUT), miso = Pin(12, Pin.IN))
sd = SDCard(spi_sd, Pin(21))
os.mount(sd, '/sd')

# gps
uart = UART(1, rx=34, tx=17)            # init with given baudrate
uart.init(9600, bits=8, parity=None, stop=1)    # init with given parameters
my_gps = MicropyGPS()

# wlan
station = network.WLAN(network.STA_IF)

if (settings['status'] == _booting):
    sensor.enable_irq_mode()
    #machine.freq(80000000)
    #machine.idle()
    settings['status'] = _sleeping


lastUpdateTimes = [0, 0, 0, 0, 0, 0]
NODE_NAME = config_lora.get_nodename()

data = {'nodes': [],
        'imu': [],
        'satellites': 0,
        'gps': [],
        'speed': "",
        'time': []}

def update_data():
    global config_lora, station, my_gps, sensor, data, intervalUpdates, lastUpdateTimes
    now = config_lora.get_millis()

    if now < lastUpdateTimes[0]: lastUpdateTimes[0] = now

    if (now - lastUpdateTimes[0] > intervalUpdates[0]):
        lastUpdateTimes[0] = now
        print('imu')
        data['imu'] = [sensor.accel, sensor.gyro, sensor.mag, sensor.temperature]
        collect_garbage()

    if (now - lastUpdateTimes[1] > intervalUpdates[1]):
        lastUpdateTimes[1] = now
        print('gps')
        stat = my_gps.updateall(uart.read(), True)
        data['satellites'] = my_gps.satellites_in_use
        data['gps'] = (my_gps.latitude_decimal(), my_gps.longitude_decimal(), my_gps.altitude) + my_gps.speed
        data['speed'] = my_gps.speed_string()
        data['time'] = my_gps.timestamp
        collect_garbage()

    if (now - lastUpdateTimes[2] > intervalUpdates[2]):
        lastUpdateTimes[2] = now
        print('wlan')
        # wlan
        if(not station.active()):
            station.active(True)

        networks = station.scan()
        wlan_nodes = []
        for net in networks:
            wlan_node = netpretty(net)
            wlan_nodes.append(wlan_node)

        data['nodes'] = wlan_nodes
        collect_garbage()

def update_display():
    global config_lora, display, data, intervalUpdates, lastUpdateTimes
    now = config_lora.get_millis()
    if now < lastUpdateTimes[3]: lastUpdateTimes[3] = now
    if (now - lastUpdateTimes[3] > intervalUpdates[3]):
        lastUpdateTimes[3] = now
        print('display')
        try:
            gps = data['gps']
            accel = data['imu'][0]
            # display information
            display.show_text('Lat.:{:04.2f}'.format(gps[0]), 0, 0, True)
            display.show_text('Long.:{:04.2f}'.format(gps[1]), 0, 10, False)
            display.show_text('Speed:{}'.format(data['speed']), 0, 20, False)
            display.show_text('A:({:03.1f},{:03.1f},{:03.1f})'.format(accel.x, accel.y, accel.z), 0, 30, False)
            display.show_text('WF:{0}|LoRa:{1}'.format(len(data['nodes']), lora.packetRssi()), 0, 40, False)
            display.show_text('Time:{:02d}:{:02d}:{:02.1f}'.format(data['time'][0],data['time'][1],data['time'][2]), 0, 50, False)
            collect_garbage()
        except:
            display.show_text('Waiting for data...', 0, 0, True)

def save_data():
    global data

    if now < lastUpdateTimes[4]: lastUpdateTimes[4] = now
    if (now - lastUpdateTimes[4] > intervalUpdates[4]):
        lastUpdateTimes[4] = now
        print('save')
        try:
            save_time = ','.join([str(x) for x in data['time']])
            save_gps  = ','.join([str(x) for x in data['gps']])
            save_imu  = data['imu'][0].xyz + data['imu'][1].xyz + data['imu'][2].xyz
            save_imu  = ','.join([str(x) for x in save_imu]) + ',' + str(data['imu'][3])
            #for net in data['nodes']:
                ### TODO: save wlan data


            fn = '/sd/data.txt'
            with open(fn,'a') as f:
                n = f.write(save_time + ',' + save_gps + ',' + save_imu + '\n') # one block
                print(n, 'bytes written')
        except:
            print('Error writing data!')

        collect_garbage()


def send_data():
    global config_lora, lora, data, intervalUpdates, lastUpdateTimes
    now = config_lora.get_millis()
    if now < lastUpdateTimes[5]: lastUpdateTimes[5] = now
    if (now - lastUpdateTimes[5] > intervalUpdates[5]):
        lastUpdateTimes[5] = now                            # timestamp the message
        print('lora')
        try:
            save_time = ','.join([str(x) for x in data['time']])
            save_gps  = ','.join([str(x) for x in data['gps']])
            save_imu  = data['imu'][0].xyz + data['imu'][1].xyz + data['imu'][2].xyz
            save_imu  = ','.join([str(x) for x in save_imu]) + ',' + str(data['imu'][3])

            msg = save_time + ',' + save_gps + ',' + save_imu
            # lora message generation
            print(msg)

            message = gen_message(NODE_NAME, msg, now)
            lora.println(msg)                                   # send message
            lora.receive()                                          # go back into receive mode
        except:
            print('Error sending data')

        collect_garbage()

while True:
    now = config_lora.get_millis()

    if(settings['status'] == _sleeping):
        station.active(False)
        #display.poweroff()

    elif(settings['status'] == _moving):
        update_display()
        update_data()
        save_data()
        send_data()
