import os

xcCompiler = "/opt/microchip/xc16/v1.11/"
imageProc = "../imageproc-lib"

assemblerFlags = "-Wa,-g" #pulled from makefile
env = Environment(PIC = '33Fj128MC706A',
                  CC = 'xc16-gcc', 
                  AS = 'xc16-as',
                  #ASFLAGS = '-p33FJ128MC706A',
                  PROGSUFFIX = '.elf', 
                  CFLAGS = '-g -omf=elf -mcpu=$PIC -D__IMAGEPROC2 -D__BOOTLOAD  -D__C7 '+assemblerFlags,
                  LINKFLAGS = "-omf=elf -mcpu=$PIC -Wl,--script=\"p33FJ128MC706A_Bootload.gld\",--heap=8192,--stack=16",
                  #include paths
                  CPPPATH=[xcCompiler+"/support/dsPIC33F/h/",
                    imageProc,
                    "src"]);
#Path to look for xc16 binaries. Can just use system path.
env.PrependENVPath('PATH',  os.environ['PATH'])

bin2hex = Builder(action = 'xc16-bin2hex $SOURCE -omf=elf',
                  suffix = 'hex', 
                  src_suffix = 'elf')
env.Append(BUILDERS = {'Hex' : bin2hex})
list = Builder(action = 'xc16-objdump -S -D $SOURCE > $TARGET', 
               suffix = 'lst', 
               src_suffix = 'elf')
env.Append(BUILDERS = {'List' : list})

imageProcLibFiles = [imageProc + "/" + n for n in [ 
    "dfmem.c",
    "gyro.c",
    "i2c_driver.c",
    "payload.c",
    "radio.c",
    "sclock.c",
    "delay.s",
    "libqw.c",
    "xl.c",
    "ppool.c",
    "spi_controller.c",
    "div16.s",
    "udiv16.s",
    "init_default.c",
    "carray.c",
    "mac_packet.c",
    "at86rf231_driver.c"]]

bootloaderFiles = ["../bootloader/imageproc/target/source/" + n for n in [
    "setadd.h",
    "setadd.c"
    ]]
    
dynaroachFiles = Glob("src/*.c")
staticLibs = [xcCompiler + "/lib/dsPIC33F/libp33FJ128MC706-elf.a", xcCompiler + "/lib/libq-elf.a"]

env.Program('dynaroach', imageProcLibFiles + dynaroachFiles + bootloaderFiles + staticLibs)

env.Hex('dynaroach')
#env.List('dynaroach') #Current throws a bunch of errors
