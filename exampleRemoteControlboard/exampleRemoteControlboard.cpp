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
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/ITorqueControl.h>

#define JOINT_ID 4
#define LENGTH 0.736   //l9 + l10 + l11 + Zcom
#define MASS 56.0
#define GRAVITY 9.8
#define FACTOR 1.75

int main(int argc, char *argv[])
{
    std::printf("WARNING: requires a running instance of RaveBot (i.e. testRaveBot or cartesianServer)\n");
    yarp::os::Network yarp;

    if (!yarp::os::Network::checkNetwork())
    {
        std::printf("Please start a yarp name server first\n");
        return 1;
    }

//right leg
    yarp::os::Property options_r;
    options_r.put("device", "remote_controlboard");
    options_r.put("remote", "/teo/rightLeg");
    options_r.put("local", "/local/teo/rightLeg");

    yarp::dev::PolyDriver dd_r(options_r);

    if (!dd_r.isValid())
    {
        std::printf("RaveBot device not available.\n");
        return 1;
    }

    yarp::dev::ITorqueControl *torq_r;
    yarp::dev::IEncoders *enc_r;
    yarp::dev::IControlMode *mode_r;

    bool ok_r = true;
    ok_r &= dd_r.view(torq_r);
    ok_r &= dd_r.view(enc_r);
    ok_r &= dd_r.view(mode_r);

    if (!ok_r)
    {
        std::printf("[warning] Problems acquiring robot interface\n");
        return 1;
    } else std::printf("[success] testAsibot acquired robot interface\n");

    int axes_r;
    enc_r->getAxes(&axes_r);
    
    //jr3 pierna derecha; Sólo se utilizará la medida de uno de los encoders relativos, y se tratarán sus datos como si procedieran del otro.
    yarp::os::Property options_right_jr3;
    options_right_jr3.put("device", "analogsensorclient");
    options_right_jr3.put("remote", "/right_jr3/ch0:o");
    options_right_jr3.put("local", "/right_jr3/ch0:i");
    
     yarp::dev::PolyDriver dd_right_jr3(options_right_jr3);
     
     if (!dd_right_jr3.isValid())
    {
        std::printf("Device not available.\n");
        return 1;
    }
    
     yarp::dev::IAnalogSensor *iAnalogSensor_r;//Interfaz genérica para sensores(gyro, conversores,), Similar a IGeneisSensor, pero controla errores
    
    if (!dd_right_jr3.view(iAnalogSensor_r))	//view(interfazDeseada) devuelve true si la interfaz de un determinado device se corresponde con la interfaz deseada
    {
        std::printf("[error] Problems acquiring interface\n");
        return 1;
    }
    
    std::printf("[success] acquired interface\n");
    
    // The following delay should avoid 0 channels and bad read
    yarp::os::Time::delay(1);

    int channels_r = iAnalogSensor_r->getChannels();	//guarda el numero de canales usados en la variable channels_r
    std::printf("Number of used channels of the right leg: %d\n", channels_r);
    
    
//left leg
    yarp::os::Property options_l;						//con opciones, especificamos dispositivo, y nombres de puertos
    options_l.put("device", "remote_controlboard");		//locales y remotos 
    options_l.put("remote", "/teo/leftLeg");
    options_l.put("local", "/local/teo/leftLeg");

    yarp::dev::PolyDriver dd_l(options_l);   //Polydriver instancia dispositivo, en este caso se llama dd

    if (!dd_l.isValid())					//isValid es verdadero si el dispositivo dd fue creado y configurado correctamente
    {
        std::printf("RaveBot device not available.\n");
        return 1;
    }

    yarp::dev::ITorqueControl *torq_l;  //Interface for control boards implementing torque control. Nos permmite medir pares con get, y setear pares con SetRefTorque
    yarp::dev::IEncoders *enc_l;		//Encoder Interface for controlboard.
    yarp::dev::IControlMode *mode_l;	//INterface for setting the control mode in control board.(le dices que parte del robot controlas(articulaciones)

    bool ok_l = true;
    ok_l &= dd_l.view(torq_l);				//&= es como una puerta AND, castea al tipo de interfaz que le indicas(torq,enc, mode)
    ok_l &= dd_l.view(enc_l);				//.view() obtiene la interfaz de un device y es verdadero si el dispositivo la implementa
    ok_l &= dd_l.view(mode_l);

    if (!ok_l)
    {
        std::printf("[warning] Problems acquiring robot interface\n");
        return 1;
    } else std::printf("[success] testAsibot acquired robot interface\n");

    int axes_l;
    enc_l->getAxes(&axes_l);								//getAxes get the number of controled axes
    
      //jr3 pierna izquierda
    yarp::os::Property options_left_jr3;
    options_left_jr3.put("device", "analogsensorclient");
    options_left_jr3.put("remote", "/left_jr3/ch1:o");
    options_left_jr3.put("local", "/left_jr3/ch1:i");
    
     yarp::dev::PolyDriver dd_left_jr3(options_left_jr3);
     
     if (!dd_left_jr3.isValid())
    {
        std::printf("Device not available.\n");
        return 1;
    }
    
     yarp::dev::IAnalogSensor *iAnalogSensor_l;//Interfaz genérica para sensores(gyro, conversores,), Similar a IGeneisSensor, pero controla tranmision de errores
    
    if (!dd_left_jr3.view(iAnalogSensor_l))	//view(interfazDeseada) devuelve true si la interfaz de un determinado device se corresponde con la interfaz deseada
    {
        std::printf("[error] Problems acquiring interface\n");
        return 1;
    }
    
    std::printf("[success] acquired interface\n");
    
    // The following delay should avoid 0 channels and bad read
    yarp::os::Time::delay(1);

    int channels_l = iAnalogSensor_l->getChannels();	//guarda el numero de canales usados en la variable channels_r
    std::printf("Number of used channels of the left leg: %d\n", channels_l);



    // hacer por rpc: set icmd cmod 4 torq
    /*
    std::vector<int> modes(axes, VOCAB_CM_POSITION);	//pones uno de los elementos/articulaciones a modo par en vez de a posicion como le resto
    modes[JOINT_ID] = VOCAB_CM_TORQUE;
    mode->setControlModes(modes.data());				//setControlModes set the current control mode for a subset of axes
    */



//While contiene lectura de ángulo de enc. relativo de la pierna derecha de TEO, obtención del par teórico
//correspondiente a partir del modelo LIPM (sin compensar riigdez estática inicial de momento), y obtención de medidas 
//de los sensores JR3 de ambas piernas

    while (true)
    {
        double q;
        enc_r->getEncoder(JOINT_ID, &q);
        double t = (-1) * LENGTH * std::sin(q * M_PI / 180.0) * MASS * GRAVITY * FACTOR;
        std::printf("angle = %f\n", q);    
        std::printf("Theorical Torque = %f\n", t);
       /* torq_l->setRefTorque(JOINT_ID, t / 2);
		  torq_r->setRefTorque(JOINT_ID, t / 2);
        * */
        
       //Parte lectura sensores jr3
       
		//lectura sensor jr3 derecho; 
        yarp::sig::Vector vector_rightjr3;						//Creamos vector para almacenar medidas del jr3 derecho
        int ret_r = iAnalogSensor_r->read(vector_rightjr3);		//el método read de la interfaz AnalogSenosr lee un vector del sensor. Los parámetros son un vectgor que contenga las ultimas medidas del sensor. Devuelve AS_OK si la lectura es buena, y AS_TIMEOUT en caso de un timeout con el sensor

        if (ret_r == yarp::dev::IAnalogSensor::AS_OK)
        {
            std::printf("Good read, got: %s\n", vector_rightjr3.toString().c_str());
        }
        else
        {
            std::printf("Bad read, error: %d\n", ret_r);
            //return 1;  // Commenting out, too draconian; on init there can be several until stabilized
        }
        
        //sensor jr3 izquierdo
           yarp::sig::Vector vector_leftjr3;						//Creamos vector para almacenar medidas del jr3 izquierdo
        int ret_l = iAnalogSensor_l->read(vector_leftjr3);		//el método read de la interfaz AnalogSenosr lee un vector del sensor. Los parámetros son un vectgor que contenga las ultimas medidas del sensor. Devuelve AS_OK si la lectura es buena, y AS_TIMEOUT en caso de un timeout con el sensor

        if (ret_l == yarp::dev::IAnalogSensor::AS_OK)
        {
            std::printf("Good read, got: %s\n", vector_leftjr3.toString().c_str());
        }
        else
        {
            std::printf("Bad read, error: %d\n", ret_l);
            //return 1;  // On init there can be several until stabilized
        }
        
        
        
        yarp::os::Time::delay(0.1);
    }

    return 0;
}
