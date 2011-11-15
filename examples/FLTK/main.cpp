/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */
/*
 $Id$

  Author(s):  Anton Deguet
  Created on: 2009-03-27

  (C) Copyright 2009-2011 Johns Hopkins University (JHU), All Rights
  Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cisstOSAbstraction/osaSleep.h>
#include <cisstOSAbstraction/osaThreadedLogFile.h>
#include <sawSartoriusScale/mtsSartoriusSerial.h>

#include "displayTask.h"
#include "displayUI.h"

int main(void)
{
    // log configuration
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::AddChannel(std::cout, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);
    // add a log per thread
    osaThreadedLogFile threadedLog("example1-");
    cmnLogger::AddChannel(threadedLog, CMN_LOG_ALLOW_ALL);
    // specify a higher, more verbose log level for these classes
    cmnLogger::SetMaskClass("mtsSartoriusSerial", CMN_LOG_ALLOW_ALL);

    // create our two tasks
    const long PeriodDisplay = 10; // in milliseconds
    mtsComponentManager * componentManager = mtsComponentManager::GetInstance();
    displayTask * displayTaskObject =
        new displayTask("Display", PeriodDisplay * cmn_ms);
    displayTaskObject->Configure();
    componentManager->AddComponent(displayTaskObject);

    mtsSartoriusSerial * scaleObject = new mtsSartoriusSerial("Sartorius", "/dev/tty.KeySerial1");
	componentManager->AddComponent(scaleObject);

    // connect the tasks
    componentManager->Connect("Display", "Scale", "Sartorius", "Scale");
    // create the tasks, i.e. find the commands
    componentManager->CreateAll();
    // start the periodic Run
    componentManager->StartAll();

    // wait until the close button of the UI is pressed
    while (1) {
        osaSleep(100.0 * cmn_ms); // sleep to save CPU
        if (displayTaskObject->GetExitFlag()) {
            std::cout << "Quitting " << std::flush;
            break;
        }
    }
    // cleanup
    componentManager->KillAll();

    osaSleep(PeriodDisplay * 2);
    while (!displayTaskObject->IsTerminated()) {
        osaSleep(PeriodDisplay);
        std::cout << "." << std::flush;
    }
    componentManager->Cleanup();
    return 0;
}
