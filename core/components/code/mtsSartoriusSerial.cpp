/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s): Anton Deguet
  Created on: 2009-04-01

  (C) Copyright 2009-2022 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <sawSartoriusScale/mtsSartoriusSerial.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>

#include <string.h> // for memcpy

CMN_IMPLEMENT_SERVICES(mtsSartoriusSerial);


mtsSartoriusSerial::mtsSartoriusSerial(const std::string & taskName,
                                       const std::string & serialPortName):
    mtsTaskContinuous(taskName, 500)
{
    this->SerialPort.SetPortName(serialPortName);
    this->SetSerialPortDefaults();
    this->SetupInterface();
}


mtsSartoriusSerial::mtsSartoriusSerial(const std::string & taskName,
                                       unsigned int serialPortNumber):
    mtsTaskContinuous(taskName, 500)
{
    this->SerialPort.SetPortNumber(serialPortNumber);
    this->SetSerialPortDefaults();
    this->SetupInterface();
}


void mtsSartoriusSerial::SetSerialPortDefaults(void)
{
    this->SerialPort.SetBaudRate(osaSerialPort::BaudRate9600);
    this->SerialPort.SetCharacterSize(osaSerialPort::CharacterSize7);
    this->SerialPort.SetParityChecking(osaSerialPort::ParityCheckingOdd);
    this->SerialPort.SetStopBits(osaSerialPort::StopBitsOne);
    this->SerialPort.SetFlowControl(osaSerialPort::FlowControlHardware);
    this->NbBytesReadSoFar = 0;
}


void mtsSartoriusSerial::SetupInterface(void)
{
    // add weight to state table
    StateTable.AddData(m_weight, "Weight");
    StateTable.AddData(m_measured_cf, "measured_cf");
    // add one interface, this will create an mtsTaskInterface
    mInterface = AddInterfaceProvided("Scale");
    if (mInterface) {
        // system commands
        mInterface->AddMessageEvents();
        mInterface->AddCommandReadState(StateTable, StateTable.PeriodStats,
                                                "period_statistics");
        // add command to access state table values to the interface
        mInterface->AddCommandReadState(this->StateTable, m_weight, "GetWeight");
        mInterface->AddCommandReadState(this->StateTable, m_measured_cf, "measured_cf");
    }
}


bool mtsSartoriusSerial::GetWeight(double & weightInGrams)
{
    CMN_LOG_CLASS_RUN_DEBUG << "GetWeight: entering method" << std::endl;
    unsigned int nbTrials;
    nbTrials = 10;
    bool enoughData = false;

    while ((!enoughData) && (nbTrials > 0)) {
        nbTrials--;
        // add whatever we can read to main buffer, read potentialy as
        // many characters as space left in buffer
        if (this->NbBytesReadSoFar < BUFFER_SIZE) {
            this->NbBytesReadSoFar +=
                this->SerialPort.Read(this->BytesReadSoFar + this->NbBytesReadSoFar,
                                      BUFFER_SIZE - this->NbBytesReadSoFar);
            CMN_LOG_CLASS_RUN_DEBUG << "GetWeight: buffer now contains "
                                    << this->NbBytesReadSoFar << " characters"
                                    << std::endl;
            enoughData = ProcessBuffer();
            weightInGrams = m_weight;
        } else {
            // looks like we have an empty buffer, empty it and hope to re-sync with scale
            CMN_LOG_CLASS_RUN_ERROR << "GetWeight: buffer is full (" << BUFFER_SIZE << " chars)" << std::endl;
            this->BytesReadSoFar[BUFFER_SIZE - 1] = '\0';
            m_measured_cf.Valid() = false;
            CMN_LOG_CLASS_RUN_DEBUG << "GetWeight: buffer contains: " << this->BytesReadSoFar << std::endl;
            this->NbBytesReadSoFar = 0;
        }
        osaSleep(1.0 * cmn_ms); // tiny sleep
    }
    if (!enoughData) {
        CMN_LOG_CLASS_RUN_VERBOSE << "GetWeight failed, not enought data" << std::endl;
        return false;
    }
    return true;
}


bool mtsSartoriusSerial::ProcessBuffer(void)
{
    CMN_LOG_CLASS_RUN_DEBUG << "ProcessBuffer: entering method" << std::endl;
	static char toProcess[16];
    // we need at least 16 characters to start any processing
    if (this->NbBytesReadSoFar < 16) {
        return false;
    }
    // look for eof as "CR" (\r) "LF" (\n)
    int index;
    int notYetProcessedBytes;
    bool eolFound;

    eolFound = false;
    index = 0;

    // try to find 'eol'
    while ((index < static_cast<int>(this->NbBytesReadSoFar - 1))
           && (!eolFound)) {
        if ((this->BytesReadSoFar[index] == '\r')
            && (this->BytesReadSoFar[index + 1] == '\n')) {
            // found eol, process if enough characters before eol
            // and in any case, remove the characters up to 'eol'
            // just found.  process is enough characters
            eolFound = true;
            CMN_LOG_CLASS_RUN_DEBUG << "ProcessBuffer: found eol in buffer at position " << index << std::endl;
            // test if the buffer already has 16 bytes
            if (index < 14) {
                mInterface->SendWarning(this->GetName() + " ProcessBuffer: partial message found in buffer, this should not happen except during initialization");
            } else {
                // get all 16 chars to be processed and removed from buffer
                memcpy(toProcess, /* destination buffer */
                       this->BytesReadSoFar + index - 14, /* begining of message from index */
                       16 * sizeof(char));
                CMN_LOG_CLASS_RUN_DEBUG << "ProcessBuffer: bytes to process \"" << toProcess << "\"" << std::endl;
                UpdateStateTable(toProcess);
            }
            // copy what's left from buffer in temporary buffer
            notYetProcessedBytes = this->NbBytesReadSoFar - (index + 2);
            if (notYetProcessedBytes < 0) {
                mInterface->SendError(this->GetName() + " ProcessBuffer: negative number of bytes in buffer");
                m_measured_cf.Valid() = false;
                return false;
            } else {
                memcpy(this->TempBuffer,
                       this->BytesReadSoFar + index + 2, /* start after end of message */
                       notYetProcessedBytes);
                // copy back and update number of bytes read
                memcpy(this->BytesReadSoFar, this->TempBuffer, notYetProcessedBytes);
                this->NbBytesReadSoFar = notYetProcessedBytes;
                return true;
            }
        }
        index++;
    }
    return false;
}


void mtsSartoriusSerial::UpdateStateTable(const const_char_pointer & buffer)
{
    if (buffer[11] == 'g') {
        // weight is stable
    }
    std::stringstream stdBuffer;
    stdBuffer << buffer + 1;
    stdBuffer >> m_weight;
    if (buffer[0] == '-') {
        m_weight = - m_weight;
    }
    m_measured_cf.Force().Z() = -9.81 * m_weight / 1000.0;
    m_measured_cf.Valid() = true;
    CMN_LOG_CLASS_RUN_DEBUG << "UpdateStateTable: found weight : " << m_weight << std::endl;
}


void mtsSartoriusSerial::Startup(void)
{
    if (!this->SerialPort.Open()) {
        mInterface->SendError(this->GetName() + " Startup: can't open serial port "
                              + this->SerialPort.GetPortName());
    }
}


void mtsSartoriusSerial::Run(void)
{
    this->ProcessQueuedCommands();
    this->GetWeight(m_weight);
    osaSleep(1.0 * cmn_ms); // way smaller than delay from scale
}


void mtsSartoriusSerial::Cleanup(void)
{
    CMN_LOG_CLASS_INIT_DEBUG << "Cleanup called" << std::endl;
    if (!this->SerialPort.Close()) {
        CMN_LOG_CLASS_INIT_ERROR << "Cleanup: can't close serial port \""
                                 << this->SerialPort.GetPortName() << "\"" << std::endl;
    }
}
