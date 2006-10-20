# Platform.py and Environ.py from http://www.iplt.org/
from Platform import *
from Environ import *
platform = GetPlatform()

if platform.IsDarwin():
    env = Environment(LIBPATH = ['#lib', '/Users/rolo/local-sfl/lib'],
                      CPPPATH = ['#inc', '/Users/rolo/local-sfl/include'])
    env.PrevLib = env.StaticLibrary
else:
    env = Environment(CC = 'gcc32', CXX = 'g++32',
                      LIBPATH = ['#lib', '/home/rolo/local-nav/lib'],
                      CPPPATH = ['#inc', '/home/rolo/local-nav/include'])
    env.PrevLib = env.SharedLibrary
    
platform.SetDebugCompilerFlags(env)

Export('env')
Export('platform')

SConscript(dirs = '.')
