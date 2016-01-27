import tornado.ioloop
import tornado.web
import time
from tornado import gen
import smbus
import struct
import sys

pairs = {0:0}

class MainHandler(tornado.web.RequestHandler):
    def get(self):
        global pairs
        for key, value in pairs.iteritems():
            self.write(str(key))
            self.write('&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;')
            self.write(str(value))
            self.write('<p>')

#@gen.engine
def schedule_func():
    global pairs
    arrayData = bus.read_i2c_block_data(address,0x00)
    print arrayData
    time = arrayData[0]
    time |= arrayData[1] << 8
    time |= arrayData[2] << 16
    time |= arrayData[3] << 24
    time = time / 1000
    temp = arrayData[4:8] 
    # time = struct.unpack('f', "".join(map(chr, time)))[0]
    temp = struct.unpack('f', "".join(map(chr, temp)))[0]
    print time
    print temp
    pairs.update({time:temp})

def make_app():
    return tornado.web.Application([
        (r"/", MainHandler),
    ])

if __name__ == "__main__":
    bus = smbus.SMBus(1)
    address = 0x04
    app = make_app()
    app.listen(8000)
    #milliseconds
    interval_ms = 15000
    main_loop = tornado.ioloop.IOLoop.instance()
    sched = tornado.ioloop.PeriodicCallback(schedule_func,interval_ms, io_loop = main_loop)
    #start your period timer
    sched.start()
    #start your loop
    main_loop.start()
    #tornado.ioloop.IOLoop.current().start()
