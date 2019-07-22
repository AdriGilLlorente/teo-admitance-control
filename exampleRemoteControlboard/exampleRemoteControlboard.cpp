// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup asibot_examples_cpp
 * \defgroup testRemoteRaveBot testRemoteRaveBot
 *
 * @brief This example connects to a running \ref testRaveBot or \ref cartesianServer module.
 *
 * <b>Legal</b>
 *
 * Copyright: (C) 2010 Universidad Carlos III de Madrid;
 *            (C) 2010 RobotCub Consortium
 *
 * Author: Juan G Victores
 *
 * Contribs: Paul Fitzpatrick and Giacomo Spigler (YARP dev/motortest.cpp example)
 *
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see license/LGPL.TXT
 *
 * <b>Building</b>
\verbatim
cd repos/asibot-main/example/cpp
mkdir build; cd build; cmake ..
make -j3
\endverbatim
 *
 * <b>Running</b>
\verbatim
./testRemoteRaveBot
\endverbatim
 *
 */

#include <cmath>
#include <cstdio>

#include <vector>

#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/Time.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/ITorqueControl.h>

#define JOINT_ID 4
#define LENGTH 1.0
#define MASS 55.0
#define GRAVITY 9.8

int main(int argc, char *argv[])
{
    std::printf("WARNING: requires a running instance of RaveBot (i.e. testRaveBot or cartesianServer)\n");
    yarp::os::Network yarp;

    if (!yarp::os::Network::checkNetwork())
    {
        std::printf("Please start a yarp name server first\n");
        return 1;
    }

    yarp::os::Property options;
    options.put("device", "remote_controlboard");
    options.put("remote", "/teo/rightLeg");
    options.put("local", "/local/teo/rightLeg");

    yarp::dev::PolyDriver dd(options);

    if (!dd.isValid())
    {
        std::printf("RaveBot device not available.\n");
        return 1;
    }

    yarp::dev::ITorqueControl *torq;
    yarp::dev::IEncoders *enc;
    yarp::dev::IControlMode *mode;

    bool ok = true;
    ok &= dd.view(torq);
    ok &= dd.view(enc);
    ok &= dd.view(mode);

    if (!ok)
    {
        std::printf("[warning] Problems acquiring robot interface\n");
        return 1;
    } else std::printf("[success] testAsibot acquired robot interface\n");

    int axes;
    enc->getAxes(&axes);

    // hacer por rpc: set icmd cmod 4 torq
    /*
    std::vector<int> modes(axes, VOCAB_CM_POSITION);
    modes[JOINT_ID] = VOCAB_CM_TORQUE;
    mode->setControlModes(modes.data());
    */

    while (true)
    {
        double q;
        enc->getEncoder(JOINT_ID, &q);
        double t = (-1) * LENGTH * std::sin(q * M_PI / 180.0) * MASS * GRAVITY;
        std::printf("t = %f\n", t);
        yarp::os::Time::delay(0.1);
    }

    return 0;
}
