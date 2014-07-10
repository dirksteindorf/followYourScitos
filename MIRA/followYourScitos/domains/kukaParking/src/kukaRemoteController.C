/*
 * Copyright (C) 2012 by
 *   MetraLabs GmbH (MLAB), GERMANY
 * and
 *   Neuroinformatics and Cognitive Robotics Labs (NICR) at TU Ilmenau, GERMANY
 * All rights reserved.
 *
 * Contact: info@mira-project.org
 *
 * Commercial Usage:
 *   Licensees holding valid commercial licenses may use this file in
 *   accordance with the commercial license agreement provided with the
 *   software or, alternatively, in accordance with the terms contained in
 *   a written agreement between you and MLAB or NICR.
 *
 * GNU General Public License Usage:
 *   Alternatively, this file may be used under the terms of the GNU
 *   General Public License version 3.0 as published by the Free Software
 *   Foundation and appearing in the file LICENSE.GPL3 included in the
 *   packaging of this file. Please review the following information to
 *   ensure the GNU General Public License version 3.0 requirements will be
 *   met: http://www.gnu.org/copyleft/gpl.html.
 *   Alternatively you may (at your option) use any later version of the GNU
 *   General Public License if such license has been publicly approved by
 *   MLAB and NICR (or its successors, if any).
 *
 * IN NO EVENT SHALL "MLAB" OR "NICR" BE LIABLE TO ANY PARTY FOR DIRECT,
 * INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES ARISING OUT OF
 * THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF "MLAB" OR
 * "NICR" HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * "MLAB" AND "NICR" SPECIFICALLY DISCLAIM ANY WARRANTIES, INCLUDING,
 * BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS
 * ON AN "AS IS" BASIS, AND "MLAB" AND "NICR" HAVE NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS OR MODIFICATIONS.
 */

/**
 * @file kukaRemoteController.C
 *    maps different messages to Gamepad buttons to fetch a Kuka robot that then follows the scitos until it's disconnected again
 *
 * @author Dirk Steindorf
 * @date   2014/05/17
 */

#include <fw/Unit.h>
#include <transform/Velocity.h>
#include <transform/Pose.h>
#include <fw/ChannelReadWrite.h>

#include "gamepad.h"
#include "gamepadEvent.h"

using namespace mira;

namespace kukaParking { 

///////////////////////////////////////////////////////////////////////////////

/**
 * send control messages to the youBot via touch of a button
 */
class kukaRemoteController : public Unit
{
MIRA_OBJECT(kukaRemoteController)

public:

	kukaRemoteController();

	template<typename Reflector>
	void reflect(Reflector& r)
	{
		Unit::reflect(r);
	}

protected:

	virtual void initialize();

	virtual void process(const Timer& timer);

private:

	// void onPoseChanged(ChannelRead<Pose2> pose);

private:

    Channel<std::string> scitosToKuka1;
    std::string followMe;
    std::string stopFollowing;

    Gamepad gamepad;
    GamepadEvent gamepadEvent;

    bool isDrivingAllowed;
    float transformSpeed;
    float rotationSpeed;

    static constexpr float SPEED_STEP = 0.1f;
    static constexpr float MAX_SPEED = 1.0f;
    static constexpr float MIN_SPEED = 0.1f;
};

///////////////////////////////////////////////////////////////////////////////

kukaRemoteController::kukaRemoteController() : Unit(Duration::milliseconds(100))
{
    followMe = "follow me";
    stopFollowing = "stop following";

    gamepad = Gamepad(1);
    isDrivingAllowed= true;
    transformSpeed = 0.0f;
    rotationSpeed = 0.0f;
}

void kukaRemoteController::initialize()
{
    scitosToKuka1 = publish<std::string>("scitosToKuka1");
    waitForService("/robot/Robot");
}

void kukaRemoteController::process(const Timer& timer)
{
    if(gamepad.isDataAvailable(&gamepadEvent))
    {
        if(gamepadEvent.isButtonReleased(BUTTON_BACK))
        {
            isDrivingAllowed = false;
            callService<void>("/robot/Robot", 
                              "emergencyStop");
        }
        if(gamepadEvent.isButtonPressed(BUTTON_START))
        {
            isDrivingAllowed = !isDrivingAllowed;
            transformSpeed = 0.0f;
            rotationSpeed = 0.0f;
            
            callService<void>("/robot/Robot", 
                              "resetMotorStop");

            callService<void>("/robot/Robot", 
                              "setVelocity", 
                              Velocity2(transformSpeed, 0.0f, rotationSpeed));
        }

        //---------------------------------------------------------------------
        // DPad 
        if(gamepadEvent.isDPadUp())
        {
            transformSpeed += SPEED_STEP;
            rotationSpeed = 0.0f;
        }
        if(gamepadEvent.isDPadDown())
        {
            transformSpeed -= SPEED_STEP;
            rotationSpeed = 0.0f;
        }
    
        if(gamepadEvent.isDPadRight())
        {
            rotationSpeed -= SPEED_STEP;
        }
        if(gamepadEvent.isDPadLeft())
        {
            rotationSpeed += SPEED_STEP;
        }
    
        //---------------------------------------------------------------------
        // color buttons 
        if(gamepadEvent.isButtonPressed(BUTTON_X))
        {
            scitosToKuka1.post(followMe);
        }
        if(gamepadEvent.isButtonPressed(BUTTON_Y)) 
        {
            scitosToKuka1.post(stopFollowing);
        }

        if(gamepadEvent.isButtonPressed(BUTTON_A))
        {
            transformSpeed = 0.0f;
            rotationSpeed = 0.0f;
        }

        if(gamepadEvent.isButtonPressed(BUTTON_B))
        {
            transformSpeed = 0.0f;
            rotationSpeed = 0.0f;
        }

        //---------------------------------------------------------------------
        // bumper and trigger 
        if(gamepadEvent.isRightTriggerPressed())
        {
            transformSpeed = MAX_SPEED;
        }

        if(gamepadEvent.isLeftTriggerPressed())
        {
            transformSpeed = MIN_SPEED;
        }
    }

    if(isDrivingAllowed)
    {
        if(transformSpeed > MAX_SPEED)
        {
            transformSpeed = MAX_SPEED;
        }
        callService<void>("/robot/Robot", 
                          "setVelocity", 
                          Velocity2(transformSpeed, 0.0f, rotationSpeed));
    }
}

///////////////////////////////////////////////////////////////////////////////

}

MIRA_CLASS_SERIALIZATION(kukaParking::kukaRemoteController, mira::Unit );
