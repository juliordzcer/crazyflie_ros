#!/usr/bin/env python3
from demo import Demo

if __name__ == '__main__':
    demo = Demo(
        [
            #x   ,   y,   z, yaw, sleep
            [0.0 ,   0, 0.5, 0, 2],
            [0.5 , -.5, 0.5, 0, 2],
            [-.5 , -.5, 0.5, 0, 2],
            [-.5 , -.5, 0.5, 0, 2],
            [0.0 , -.5, 0.5, 0, 0],
        ]
    )
    demo.run()
