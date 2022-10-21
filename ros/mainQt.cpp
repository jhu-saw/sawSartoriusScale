/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2017-12-04

  (C) Copyright 2017-2022 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


#include <sawSartoriusScale/mtsSartoriusSerial.h>

#include <cisstCommon/cmnCommandLineOptions.h>
#include <cisstCommon/cmnQt.h>
#include <cisstMultiTask/mtsTaskManager.h>
#include <cisstMultiTask/mtsSystemQtWidget.h>
#include <cisstParameterTypes/prmForceCartesianGetQtWidget.h>

#include <QApplication>

#include <cisst_ros_crtk/mts_ros_crtk_bridge_provided.h>
#include <ros/ros.h>

int main(int argc, char * argv[])
{
    // log configuration
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskFunction(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    cmnLogger::AddChannel(std::cout, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);

    // create ROS node handle
    ros::init(argc, argv, "sartorius_scale_ros", ros::init_options::AnonymousName);
    ros::NodeHandle rosNodeHandle;

    // parse options
    cmnCommandLineOptions options;
    std::string port;
    double rosPeriod = 20.0 * cmn_ms;

    options.AddOptionOneValue("s", "serial-port",
                              "serial port (e.g. /dev/ttyUSB0, COM...)",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &port);
    options.AddOptionNoValue("l", "log-serial",
                             "log all serial port read/writes in cisstLog.txt");
    options.AddOptionOneValue("p", "ros-period",
                              "period in seconds to read all components and publish (default 0.02, 20 ms, 50Hz).  There is no point to have a period higher than the scale's period",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &rosPeriod);
    options.AddOptionNoValue("D", "dark-mode",
                             "replaces the default Qt palette with darker colors");

    typedef std::list<std::string> managerConfigType;
    managerConfigType managerConfig;
    options.AddOptionMultipleValues("m", "component-manager",
                                    "JSON file to configure component manager",
                                    cmnCommandLineOptions::OPTIONAL_OPTION, &managerConfig);

    // check that all required options have been provided
    std::string errorMessage;
    if (!options.Parse(argc, argv, errorMessage)) {
        std::cerr << "Error: " << errorMessage << std::endl;
        options.PrintUsage(std::cerr);
        return -1;
    }
    std::string arguments;
    options.PrintParsedArguments(arguments);
    std::cout << "Options provided:" << std::endl << arguments << std::endl;

    // log configuration
    if (options.IsSet("log-serial")) {
        std::cout << "Adding log for all serial port read/writes" << std::endl;
        cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
        cmnLogger::SetMaskFunction(CMN_LOG_ALLOW_ALL);
        cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
        cmnLogger::SetMaskClass("mtsSartoriusSerial", CMN_LOG_ALLOW_ALL);
        cmnLogger::SetMaskClass("osaSerialPort", CMN_LOG_ALLOW_ALL);
        cmnLogger::AddChannel(std::cerr, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);
    }

    // create a Qt user interface
    QApplication application(argc, argv);
    cmnQt::QApplicationExitsOnCtrlC();
    if (options.IsSet("dark-mode")) {
        cmnQt::SetDarkMode();
    }

    // organize all widgets in a tab widget
    QTabWidget * tabWidget = new QTabWidget;

    // create the components
    mtsSartoriusSerial * scale = new mtsSartoriusSerial("Sartorius", port);

    // add the components to the component manager
    mtsManagerLocal * componentManager = mtsComponentManager::GetInstance();
    componentManager->AddComponent(scale);

    // Qt
    prmForceCartesianGetQtWidgetComponent * forceWidget
        = new prmForceCartesianGetQtWidgetComponent("measured_cf");
    forceWidget->Configure();
    componentManager->AddComponent(forceWidget);
    componentManager->Connect(forceWidget->GetName(), "Component",
                              scale->GetName(), "Scale");
    tabWidget->addTab(forceWidget, "State");

    mtsSystemQtWidgetComponent * systemWidget
        = new mtsSystemQtWidgetComponent("System");
    systemWidget->Configure();
    componentManager->AddComponent(systemWidget);
    componentManager->Connect(systemWidget->GetName(), "Component",
                              scale->GetName(), "Scale");
    tabWidget->addTab(systemWidget, "System");

    // ROS CRTK bridge
    mts_ros_crtk_bridge_provided * crtk_bridge
        = new mts_ros_crtk_bridge_provided("sartorius_scale_crtk_bridge", &rosNodeHandle);
    crtk_bridge->bridge_interface_provided(scale->GetName(), "Scale",
                                           rosPeriod);
    componentManager->AddComponent(crtk_bridge);
    crtk_bridge->Connect();

    // custom user component
    if (!componentManager->ConfigureJSON(managerConfig)) {
        CMN_LOG_INIT_ERROR << "Configure: failed to configure component-manager, check cisstLog for error messages" << std::endl;
        return -1;
    }

    // create and start all components
    componentManager->CreateAllAndWait(5.0 * cmn_s);
    componentManager->StartAllAndWait(5.0 * cmn_s);

    // run Qt user interface
    tabWidget->show();
    application.exec();

    // stop all logs
    cmnLogger::Kill();

    // stop ROS node
    ros::shutdown();

    // kill all components and perform cleanup
    componentManager->KillAllAndWait(5.0 * cmn_s);
    componentManager->Cleanup();

    return 0;
}
