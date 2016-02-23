import tornado.ioloop
import tornado.web
import time
import smbus
import struct
import sys

pairs = {0:0}
interval_ms = 15000

# set up i2c connection
address = 0x04
bus = smbus.SMBus(1)

class MainHandler(tornado.web.RequestHandler):
    def get(self):
        self.render("index.html")

    def post(self):
        self.render("index.html")
        print self.get_body_argument("temp")

def schedule_func():
    # request to i2c
    arrayData = bus.read_i2c_block_data(address,0x00)
    # extract time from the first 32 bit
    time = arrayData[0]
    time |= arrayData[1] << 8
    time |= arrayData[2] << 16
    time |= arrayData[3] << 24
    time = time / 1000
    # extract temp from the next 32 bit
    temp = arrayData[4:8] 
    temp = struct.unpack('f', "".join(map(chr, temp)))[0]
    # update with new data 
    pairs.update({time:temp})

def main():
    # make main IOLoop
    main_loop = tornado.ioloop.IOLoop.instance()
    sched = tornado.ioloop.PeriodicCallback(schedule_func,interval_ms, io_loop = main_loop)
    # start periodic function
    sched.start()
    # start web 
    app = tornado.web.Application([
        (r"/", MainHandler),
    ])
    app.listen(80)
    main_loop.start()

if __name__ == "__main__":
    main()
