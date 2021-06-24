/* Copyright (c) 2017 Michael Ammann
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "serial_pipe.h"
#include <cstdio>

SerialPipe::SerialPipe(PinName tx, PinName rx, int baudrate, int rxSize, int txSize) :
            _SerialPipeBase(tx, rx, baudrate),
            _pipeRx( (rx!=NC) ? rxSize : 0),
            _pipeTx( (tx!=NC) ? txSize : 0)
{
    if (rx!=NC) {
        _SerialPipeBase::attach(callback(this, &SerialPipe::rxIrqBuf), Serial::RxIrq);
    }
}

SerialPipe::~SerialPipe(void)
{
    _SerialPipeBase::attach(NULL, Serial::RxIrq);
    _SerialPipeBase::attach(NULL, Serial::TxIrq);
}

// tx channel
int SerialPipe::writeable(void)
{
    return _pipeTx.free();
}

int SerialPipe::putc(int c)
{
    c = _pipeTx.putc(c);
    txStart();
    return c;
}

int SerialPipe::put(const void* buffer, int length, bool blocking)
{
    int count = length;
    const char* ptr = (const char*)buffer;
    if (count) {
        do {
            int written = _pipeTx.put(ptr, count, false);
            if (written) {
                ptr += written;
                count -= written;
                txStart();
            }
            else if (!blocking) {
                /* nothing / just wait */
                break;
            }
        }
        while (count);
    }

    return (length - count);
}

void SerialPipe::txCopy(void)
{
    while (_SerialPipeBase::writeable() && _pipeTx.readable()) {
        char c = _pipeTx.getc();
        _SerialPipeBase::putc(c);
    }
}

void SerialPipe::txIrqBuf(void)
{
    txCopy();
    // detach tx isr if we are done
    if (!_pipeTx.readable()) {
        _SerialPipeBase::attach(NULL, Serial::TxIrq);
    }
}

void SerialPipe::txStart(void)
{
    // disable the tx isr to avoid interruption
    _SerialPipeBase::attach(NULL, Serial::TxIrq);
    txCopy();
    // attach the tx isr to handle the remaining data
    if (_pipeTx.readable()) {
        _SerialPipeBase::attach(callback(this, &SerialPipe::txIrqBuf), Serial::TxIrq);
    }
}

// rx channel
int SerialPipe::readable(void)
{
    return _pipeRx.readable();
}

int SerialPipe::getc(void)
{
    if (!_pipeRx.readable()) {
        return EOF;
    }

    return _pipeRx.getc();
}

int SerialPipe::get(void* buffer, int length, bool blocking)
{
    return _pipeRx.get((char*)buffer,length,blocking);
}

void SerialPipe::rxIrqBuf(void)
{
    while (_SerialPipeBase::readable())
    {
        char c = _SerialPipeBase::getc();
        if (_pipeRx.writeable()) {
            _pipeRx.putc(c);
        } else {
            /* overflow */
        }
    }
}
