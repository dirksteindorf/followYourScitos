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
 * @file singleKukaParking.C
 *    parking procedure for only one youBot
 *
 * @author Dirk Steindorf
 * @date   2014/05/16
 */

#include <fw/Unit.h>
#include <transform/Pose.h>
#include <fw/ChannelReadWrite.h>

using namespace mira;

namespace kukaParking { 

///////////////////////////////////////////////////////////////////////////////

/**
 * parking procedure for only one youBot
 */
class singleKukaParking : public Unit
{
MIRA_OBJECT(singleKukaParking)

public:

	singleKukaParking();

	template<typename Reflector>
	void reflect(Reflector& r)
	{
		Unit::reflect(r);
	}

protected:

	virtual void initialize();

	virtual void process(const Timer& timer);

private:

    void onKuka1(ChannelRead<std::string> data);
    void onPilotEvent(ChannelRead<std::string> data);

private:

    Channel<std::string> scitosToKuka1;
    Channel<std::string> scitosToKuka2;

    std::string followMe;
    std::string stopFollowing;
    std::string scitosHere;

    bool kuka1answered;

    bool kuka1follows;

    bool getKuka1;

    bool isInitialState;

    Pose2 workstation1;
    Pose2 workstation2;
    Pose2 workstation3;
    Pose2 workstation4;

    Pose2 sideline1;
    Pose2 sideline2;

    float translationTolerance; // meter
    float rotationTolerance; // radian

    int cycle;
};

///////////////////////////////////////////////////////////////////////////////

singleKukaParking::singleKukaParking() : Unit(Duration::milliseconds(100))
{
    followMe = "follow me";
    stopFollowing = "stop following";
    scitosHere = "scitos here";

    kuka1answered = false;

    kuka1follows = false;

    getKuka1 = false;

    isInitialState = true;

    workstation1 = Pose2(2.45f, 0.0f, 2700.0f);
    workstation2 = Pose2(3.35f, 0.0f, 270.0f);
    workstation3 = Pose2(3.35f, -4.2f, 90.0f);
    workstation4 = Pose2(2.45f, -4.2f, 90.0f);

    // TODO: change position for sideline to a more central point
    sideline1 = Pose2(1.55f, 0.0f, 180.0f);
    sideline2 = Pose2(1.55f, -4.2f, 180.0f);

    translationTolerance = 0.1f;
    rotationTolerance = 0.1f;

    cycle = 0;

}

void singleKukaParking::initialize()
{
    scitosToKuka1 = publish<std::string>("scitosToKuka1");
    scitosToKuka2 = publish<std::string>("scitosToKuka2");

	subscribe<std::string>("kuka1ToScitos", &singleKukaParking::onKuka1);
	subscribe<std::string>("/navigation/PilotEvent", 
	                        &singleKukaParking::onPilotEvent);

    waitForService("/navigation/Pilot");
}

void singleKukaParking::process(const Timer& timer)
{
    if(!kuka1answered)
    {
        scitosToKuka1.post(scitosHere);
    }

    if(kuka1answered && isInitialState)
    {
        isInitialState = false;
        getKuka1 = true;
        callService<void>("/navigation/Pilot", "setGoal", workstation1, 
                            translationTolerance, rotationTolerance);

    }
}
    
//------------------------------------------------------------------------------
// callback for Kuka 1
void singleKukaParking::onKuka1(ChannelRead<std::string> data)
{
    if(data->value() == "heard you")
    {
        kuka1answered = true;
    }
    if(data->value() == "searching")
    {
    }
    if(data->value() == "marker found")
    {
        kuka1follows = true;
        // bring Kuka 1 to workstation
        switch(cycle)
        {
            case 0:
                callService<void>("/navigation/Pilot", "setGoal", workstation4, 
                                    translationTolerance, rotationTolerance);
                cycle = 1;
                break;
            case 1:
                callService<void>("/navigation/Pilot", "setGoal", workstation2, 
                                    translationTolerance, rotationTolerance);
                cycle = 2;
                break;
            case 2:
                callService<void>("/navigation/Pilot", "setGoal", workstation3, 
                                    translationTolerance, rotationTolerance);
                cycle = 3;
                break;
            case 3:
                callService<void>("/navigation/Pilot", "setGoal", workstation1, 
                                    translationTolerance, rotationTolerance);
                cycle = 0;
                break;
        }
    }
    if(data->value() == "stopping")
    {
        kuka1follows = false;
        // move aside
        if(cycle == 0 || cycle == 2)
        {
            callService<void>("/navigation/Pilot", "setGoal", sideline2, 
                                translationTolerance, rotationTolerance);
        }
        if(cycle == 1 || cycle == 3)
        {
            callService<void>("/navigation/Pilot", "setGoal", sideline1, 
                                translationTolerance, rotationTolerance);
        }
    }
    if(data->value() == "done")
    {
        // fetch Kuka
        switch(cycle)
        {
            case 0:
                callService<void>("/navigation/Pilot", "setGoal", workstation1, 
                                    translationTolerance, rotationTolerance);
                break;
            case 1:
                callService<void>("/navigation/Pilot", "setGoal", workstation4, 
                                    translationTolerance, rotationTolerance);
                break;
            case 2:
                callService<void>("/navigation/Pilot", "setGoal", workstation2, 
                                    translationTolerance, rotationTolerance);
                break;
            case 3:
                callService<void>("/navigation/Pilot", "setGoal", workstation3, 
                                    translationTolerance, rotationTolerance);
                break;
        }
    }
}


//------------------------------------------------------------------------------
// callback for driving state of the Scitos 
void singleKukaParking::onPilotEvent(ChannelRead<std::string> data)
{
    if(data->value() == "GoalReached")
    {
        if(kuka1follows)
        {
            scitosToKuka1.post(stopFollowing);
        }
        else // driving to fetch a kuka
        {
            if(getKuka1)
            {
                std::cout<<"Kuka 1 soll folgen"<<std::endl;
                scitosToKuka1.post(followMe);
            }
        }
    }
}

///////////////////////////////////////////////////////////////////////////////

}

MIRA_CLASS_SERIALIZATION(kukaParking::singleKukaParking, mira::Unit );
