"""
helper functions for scons

This file is part of the image processing library & toolbox
for electron microscopy, licensed under the GPL. Please see
the accompanying LICENSE file for more information. The iplt
homepage is http://www.iplt.org

Author: Ansgar Philippsen

Adapted by Roland Philippsen (rolo) for theater robot project,
http://asl.epfl.ch/
"""

import SCons
import sys, os, string, copy
import types
import shutil

def envMerge(env,other):
    "merge two environments (in fact merge dicts for some entries)"
    kw = ['CPPPATH', 'CCFLAGS', 'CXXFLAGS', 'LIBPATH', 'LIBS', 'LINKFLAGS']
    for key in kw:
        # define first list
        list1 = []
        if(key in env._dict):
            v1 = env._dict[key]
            if type(v1) is types.ListType:
                list1 = v1
            else:
                list1.append(v1)
        # define second list
        list2 = []
        if(key in other._dict):
            v2 = copy.deepcopy(other._dict[key])
            if type(v2) is types.ListType:
                list2 = v2
            else:
                list2.append(v1)
        # remove duplicate entries
        for i2 in list2:
            if (i2 not in list1):
                list1.append(i2)
        #env._dict[key] = env.Flatten(list1)
        env._dict[key] = list1

def envInstallExec(env,target):
    "install exec files"
    t=env.Install(dir = env._stagedir + "/bin/", source = target)
    env.Default(t)
    if(not type(t) is types.ListType):
        t=[t]
    return t

def envInstallLib(env,target):
    "install libraries"
    t=env.Install(dir = env._stagedir + "/lib/", source = target)
    env.Default(t)
    if(not type(t) is types.ListType):
        t=[t]
    return t

def envInstallHeader(env,target,location):
    "install header files"
    t=env.Install(dir = env._stagedir + "/include/iplt/"+location, source = target)
    env.Default(t)
    if(not type(t) is types.ListType):
        t=[t]
    return t

def envInstallPymod(env,target,location):
    "install python modules"
    t=env.Install(dir = env._stagedir + "/lib/pymod/iplt/"+location, source = target)
    env.Default(t)
    if(not type(t) is types.ListType):
        t=[t]
    return t

def envInstallPlugin(env,target):
    "install plugin"
    t=env.Install(dir = env._stagedir + "/lib/plugins/", source = target)
    env.Default(t)
    if(not type(t) is types.ListType):
        t=[t]
    return t
    # TODO add algorithm into list that is parsed by init

#def envReplaceOption(env, key, oldOpt, newOpt):
#    import re;
#    s = env.get(key, None)
#    re_pattern = r"(?<=\s)%s(?!\S)" % (re.escape(oldOpt))
#    re_repl = newOpt
#    re_string = s
#    print "**************************************************"
#    print "envReplaceOption()"
#    print "  pattern = %s" % re_pattern
#    print "  repl = %s" % re_repl
#    print "  string = %s" % re_string
#    print "**************************************************"
#    #
#    # debug notes on Mac OS 10.3.6 with Python 2.3 (rolo)
#    #
#    # pattern = (?<=\s)\-shared(?!\S)
#    #  (?<=\s)  : positive lookbehind assertion on any whitespace character
#    #  \-shared : why the backslash?
#    #  (?!\S)   : negative lookahead assertion on any non-whitespace char
#    # repl = -dynamiclib
#    # string = $LINKFLAGS -dynamiclib
#    #
#    # so actually, don't need to replace anything!
#    #
#    env[key] = re.sub(re_pattern, re_repl, re_string)
#    #env[key] = re.sub(r"(?<=\s)%s(?!\S)" % (re.escape(oldOpt)), newOpt, s)

def envTestRunner(self, env, target, source):
    import os
    cwd_save = os.getcwd()
    (dir,file) = os.path.split(str(target[0]))
    os.chdir(dir)
    print "UnitTest: running " + file + " in " + dir
    os.system("./"+file)
    os.chdir(cwd_save)
    print "UnitTest: done"

def envGetRevision(self):
    import os
    cwd_save = os.getcwd()
    os.chdir(str(self.Dir("#").abspath))
    p=os.popen("svn info | grep Revision")
    os.chdir(cwd_save)
    pp=p.read()
    if(len(pp)==0):
        return "0"
    field = string.split(pp)
    p.close()
    return field[1];

def InstallWildFile(arg, dirname, names):
    # filter out
    for n in names:
        if(n[0]=='.'):
            names.remove(n)

    if(arg[0]=='/'):
        dest=arg
    else:
        dest=os.path.join(arg,dirname)

    try:
        os.makedirs(dest)
    except:
        pass

    for n in names:
        f=os.path.join(dirname,n)
        if(os.path.isfile(f)):
            #print "installing %s to %s"%(f,dest)
            shutil.copy2(f,dest)
            pass
    
def envInstallWild(self,dest,src):
    os.path.walk(str(Dir(src)),InstallWildFile,str(Dir(dest)))

def envSetStageDir(self,dir):
    self._stagedir=dir

# put into environment
SCons.Environment.Environment.Merge = envMerge
SCons.Environment.Environment.InstallExec = envInstallExec
SCons.Environment.Environment.InstallLib = envInstallLib
SCons.Environment.Environment.InstallHeader = envInstallHeader
SCons.Environment.Environment.InstallPymod = envInstallPymod
SCons.Environment.Environment.InstallPlugin = envInstallPlugin
#SCons.Environment.Environment.ReplaceOption = envReplaceOption
SCons.Environment.Environment.TestRunner = envTestRunner
SCons.Environment.Environment.GetRevision = envGetRevision
SCons.Environment.Environment.InstallWild = envInstallWild
SCons.Environment.Environment.SetStageDir = envSetStageDir

# Some more extensions to Environment class...

def envStageHeader(self, subdir, hlist):
    return self.Install('#inc/' + subdir, hlist)
SCons.Environment.Environment.StageHeader = envStageHeader

def envStageLibrary(self, llist):
    return self.Install('#lib', llist)
SCons.Environment.Environment.StageLibrary = envStageLibrary

def envStageBinary(self, flist):
    return self.Install('#bin', flist)
SCons.Environment.Environment.StageBinary = envStageBinary
