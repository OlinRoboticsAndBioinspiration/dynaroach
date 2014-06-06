#!/usr/bin/python
#
# Copyright (c) 2012, Regents of the University of California
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# - Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# - Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# - Neither the name of the University of California, Berkeley nor the names
#   of its contributors may be used to endorse or promote products derived
#   from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
#
# Run test suite
#
# by Fernando L. Garcia Bermudez
#
# v.0.1
#
# Revisions:
#  Fernando L. Garcia Bermudez      2012-8-20    Initial release
#

import sys, traceback
import dynaroach

RADIO_DEV_NAME  = '/dev/ttyUSB1'# or 'COMx'
RADIO_BAUD_RATE = 230400
RADIO_DEST_ADDR = '\x00\x15'

def main():
    rb = dynaroach.DynaRoach(RADIO_DEV_NAME,            \
                              baud_rate=RADIO_BAUD_RATE, \
                              dest_addr=RADIO_DEST_ADDR  )

    print('\nI: Testing radio communication:')
    rb.echo()

    print('\nI: Testing gyroscopes:\n')
    rb.test_gyro

    print('\nI: Testing accelerometers:\n')
    rb.test_accel

    print('\nI: Testing flash memory:\n')
    rb.test_dflash
    
    print('\nI: Testing battery:\n')
    rb.test_batt

    print('\nI: Testing motor:\n')
    rb.test_motor

    print('\nI: Testing sma:\n')
    rb.test_sma
    
    rb.__del__()

### Exception handling

if __name__ == '__main__':
    try:
        main()
        sys.exit('passed all the tests')
    except SystemExit as e:
        print('\nI: SystemExit: ' + str(e))
    except KeyboardInterrupt:
        print('\nI: KeyboardInterrupt')
    except Exception as e:
        print('\nE: Unexpected exception!\n' + str(e))
        traceback.print_exc()
