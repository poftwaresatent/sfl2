"""
platform identification tool for iplt, based on scons

This file is part of the image processing library & toolbox
for electron microscopy, licensed under the GPL. Please see
the accompanying LICENSE file for more information. The iplt
homepage is http://www.iplt.org

Author: Ansgar Philippsen

Adapted by Roland Philippsen (rolo) for theater robot project,
http://asl.epfl.ch/
"""

import copy, sys, os, string

#abstract base class
class Platform:
    def __init__(self,osname,pfname):
        self._osname = osname
        self._pfname = pfname

    def IsPosix(self):
        return False;

    def IsLinux(self):
        return False;

    def IsDarwin(self):
        return False;

    def IsIrix(self):
        return False;

    def IsWin32():
        return False;
    
    def SetCompilerFlags(self, env):
        "appends platform specific flags to environment"
        pass

    def SetDebugCompilerFlags(self, env):
        "appends platform specific flags with debg symbols to environment"
        pass

    def SetWXInfo(self, env):
        "appends wx params to environment"
        pass;

    def SetPythonInfo(self, env):
        "appends python params to environment"
        pass

    def SetPymodBuilder(self,env, ver):
        "sets Python module builder named 'Pymod'"
        pass

    def SetPluginInfo(self,env):
        "sets stuff for plugins"
        pass

    def ExtendSharedLibraryPath(self,path):
        "add the given path to the list of shared library locations"
        pass

    def SetGLStuff(self, env):
        "set linker flags for using OpenGL / Mesa etc (hack!)"
        pass
    

class PosixPlatform(Platform):
    def IsPosix(self):
        return True;

    def SetWXInfo(self, env):
        env.ParseConfig('wx-config --cxxflags')
        env.ParseConfig('wx-config --libs')
        
    def oldSetWXInfo(self, env):
        p=os.popen("wx-config --cxxflags")
        flaglist = string.split(p.read())
        p.close()
        for i in flaglist:
            if(i[:2]=='-D'):
                env.Prepend(CCFLAGS = i )
            elif(i[:2]=='-I'):
                env.Prepend(CPPPATH = i[2:])
                    
        p=os.popen("wx-config --libs")
        liblist = string.split(p.read())
        p.close()
        for i in liblist:
            if(i[:2]=='-l'):
                env.Prepend(LIBS = [ i[2:] ])
            elif(i[:2]=='-L'):
                env.Prepend(LIBPATH = [ i[2:] ])
            else:
                env.Prepend(LINKFLAGS = [i] )

    def AddPythonInclude(self, env, ver):
	if(ver==""):
            (maj,min,d,d,d)=sys.version_info
            dname = "%s/include/python%d.%d"%(sys.prefix,maj,min)
	else:
            dname = ""
            plist = env.Split(env["CPPPATH"]) + ["/usr/include"]
            for p in plist:
                id = p + "/python%s"%(ver)
                if(os.access(id,os.F_OK)):
                    dname = id
            if(dname==""):
                sys.exit("fatal error: python include dir for version %s not found!"%ver)

            
        env.Append(CPPPATH = dname)

    def AddPythonLib(self, env, ver):
	if(ver==""):
            (maj,min,d,d,d)=sys.version_info
            lname = "python%d.%d"%(maj,min)
	else:
            lname = ""
            plist = env.Split(env["CPPPATH"]) + ["/usr/include"]
            for p in plist:
                id = p + "/python%s"%(ver)
                if(os.access(id,os.F_OK)):
                    lname = "python%s"%ver

        env.Prepend(LIBS = [lname])

    def SetPymodBuilder(self,env):
        "on posix systems based on shared library"
        pymod_builder = copy.copy(env['BUILDERS']['SharedLibrary'])
        pymod_builder.prefix = ''
        env.Append(BUILDERS = {'Pymod' : pymod_builder})

    def SetPluginInfo(self,env):
        env.Append(LIBS = 'iplt')

    def ExtendSharedLibraryPath(self,path):
        os.putenv("LD_LIBRARY_PATH",os.getenv("LD_LIBRARY_PATH")+":"+path)

# linux specific
class LinuxPlatform(PosixPlatform):
    def IsLinux(self):
        return True;

    def GetGCCVersion(self):
        p=os.popen("gcc --version")
        a=string.split(p.read())
        return string.split(a[2],".")

    def SetCompilerFlags(self, env):
        PosixPlatform.SetCompilerFlags(self,env)
        env.Append(CPPFLAGS = ['-DLINUX'])

    def SetDebugCompilerFlags(self, env):
        self.SetCompilerFlags(env)
        env.Append(CPPFLAGS = ['-g', '-Wall'])

    def SetGLStuff(self, env):
        env.Append(LIBS = ['glut', 'GLU', 'GL', 'Xi', 'Xmu'])
        env.Append(LIBPATH = ['/usr/X11R6/lib'])



# osx specific
class DarwinPlatform(PosixPlatform):
    def IsDarwin(self):
        return True;

    def SetCompilerFlags(self, env):
        PosixPlatform.SetCompilerFlags(self, env)
        env.Append(CPPFLAGS = ['-DOSX', '-Wno-long-double'])
        env.Replace(SHCCFLAGS = "-dynamic")
        env.Replace(SHCXXFLAGS = "-dynamic")
        env.Replace(SHLINKFLAGS = "-dynamiclib -undefined suppress -Wl,-flat_namespace")

    def SetDebugCompilerFlags(self, env):
        self.SetCompilerFlags(env)
        env.Append(CPPFLAGS = ['-g', '-Wall'])
        
    def SetPythonInfo(self, env, ver):
        # ver is ignored
        dname = "/System/Library/Frameworks/Python.framework/Headers"
        env.Append(CPPPATH = dname)
        env.Append(LINKFLAGS = ['-framework','Python'])
        env.ReplaceOption("SHLINKFLAGS", "-dynamiclib", "-bundle")
        env.Replace(SHLIBSUFFIX = ".so")

    def SetPymodBuilder(self,env):
        pymod_builder = copy.copy(env['BUILDERS']['SharedLibrary'])
        pymod_builder.prefix = ''
        env.Append(BUILDERS = {'Pymod' : pymod_builder})

    def SetPluginInfo(self,env):
        env.Append(LIBS = 'iplt')
        env.ReplaceOption("SHLINKFLAGS", "-dynamiclib", "-bundle")
        env.Replace(SHLIBSUFFIX = ".so")

    def SetWXInfo(self, env):
        fw = ""
        p=os.popen("wx-config --cxxflags")
        flaglist = string.split(p.read())
        p.close()
        for i in flaglist:
            if(i[:2]=='-D'):
                env.Prepend(CCFLAGS = " "+i+" " )
            elif(i[:2]=='-I'):
                env.Prepend(CPPPATH = i[2:])
                    
        p=os.popen("wx-config --libs")
        liblist = string.split(p.read())
        p.close()
        nextfw=0
        for i in liblist:
            if(nextfw==1):
                fw += " -framework %s"%(i)
                nextfw=0
            else :
                if(i[:2]=='-l'):
                    env.Prepend(LIBS = [ i[2:] ])
                elif(i[:2]=='-L'):
              	    env.Prepend(LIBPATH = [ i[2:] ])
                elif(i=='-framework'):
                    nextfw=1
                else:
                    env.Prepend(LINKFLAGS = [i] )

        env.Append(LINKFLAGS = [fw])

    def SetGLStuff(self, env):
	#    env.Append(LIBS = ['objc'])
	#    env.Append(LINKFLAGS = ['-framework OpenGL -framework GLUT'])
	#    env.Append(LIBS = ['objc'])
	#    env.Append(_LIBFLAGS = ['-framework OpenGL -framework GLUT'])
        env.Append(LIBS = ['objc'])
        env.Append(LINKFLAGS = '-framework OpenGL -framework GLUT')



# sgi specific
class IrixPlatform(PosixPlatform):
    def IsIrix(self):
        return True;

    def SetCompilerFlags(self, env):
        PosixPlatform.SetCompilerFlags(self,env)
        #env.Append(CPPFLAGS = "-DIRIX -LANG:std -OPT:Olimit=0 -OPT:IEEE_NaN_inf=ON -no_auto_include -ptused -prelink -ptv -FE:eliminate_duplicate_inline_copies -FE:template_in_elf_section -n32 -mips3 -woff 1152,1174,1183,1355,1460,3333,3506")
        env.Append(CPPFLAGS = "-DIRIX -LANG:std -n32 -no_auto_include -mips3 -woff 1152,1174,1183,1355,1460,3333,3506")
	env.Append(LIBS = ['m','pthread'])

    def SetDebugCompilerFlags(self, env):
        self.SetCompilerFlags(env)

    def SetWXInfo(self, env):
        p=os.popen("wx-config --cxxflags")
        flaglist = string.split(p.read())
        p.close()
        for i in flaglist:
            if(i[:2]=='-D'):
                env.Append(CCFLAGS = " "+i+" " )
            elif(i[:2]=='-I'):
                env.Append(CPPPATH = i[2:])
                    
        p=os.popen("wx-config --static --libs")
        liblist = string.split(p.read())
        p.close()
        for i in liblist:
            if(i[:2]=='-l'):
                env.Append(LIBS = [ i[2:] ])
            elif(i[:2]=='-L'):
                env.Append(LIBPATH = [ i[2:] ])
	    elif(i[:1]=='/'):
	        env.Append(STATICLIBS = [i])
            else:
                env.Append(LINKFLAGS = [i] )





# win32 specific
class Win32Platform(Platform):
    def IsWin32():
        return True;
    
    def SetCompilerFlags(self, env):
        env.Append(CPPFLAGS = "-DWIN32")

    def SetDebugCompilerFlags(self, env):
        self.SetCompilerFlags(env)

    def SetWXInfo(self, env):
        pass;

    def SetPythonInfo(self, env, ver):
        # ver is ignored
        dname = "%s/include/"%(sys.prefix)
        env.Append(CPPPATH = Dir(dname))

# factory
def GetPlatform():
    import sys, os, string

    osname = string.lower(os.name)
    pfname = string.lower(sys.platform)

    if(osname == "posix"):
        if(string.find(pfname,"linux")>=0):
            print "Identified linux platform"
            return LinuxPlatform(osname,pfname)
        elif(string.find(pfname,"darwin")>=0):
            print "Identified osx platform"
            return DarwinPlatform(osname,pfname)
        elif(string.find(pfname,"irix")>=0):
            print "Identified Irix platform"
            return IrixPlatform(osname,pfname)
        else:
            print "Identified generic posix platform"
            return PosixPlatform(osname,pfname)
    elif((osname == "nt") & (pfname== "win32")):
        print "Identified Win32 platform"
        return Win32Platform(osname,pfname)
    else:
        raise "Unknown Platform (%s,%s)"%(osname,pfname)

