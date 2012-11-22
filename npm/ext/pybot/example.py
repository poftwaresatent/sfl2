import pynpm

def update(robot, info):
    print 'hello from python update of robot %s' % robot
    speed = (0.2, 0.3)
    return speed

def init(robot, server):
    print 'hello from python init of robot %s' % robot
    server.addLine( 0.2, -0.3,  0.2,  0.3)
    server.addLine( 0.2,  0.3, -0.2,  0.3)
    server.addLine(-0.2,  0.3, -0.2, -0.3)
    server.addLine(-0.2, -0.3,  0.2, -0.3)
    server.defineDiffDrive(0.5, 0.2)
    server.defineLidar(0,  0.15,   0, 181, 8, -90, 180)
    server.defineLidar(0, -0.15, 180, 181, 8, -90, 180)

def create(name):
    print 'hello from python create for robot %s' % name
    robot = name
    return (robot, update)
