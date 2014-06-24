
MIRA_REQUIRE_PACKAGE(kukaParking)

###############################################################################

MIRA_ADD_DOCUMENTATION(kukaParking
	DIRS
		doc
		include
	DEPENDS
		#RobotDataTypes
)

###############################################################################

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_LIST_DIR}/include)

###############################################################################

MIRA_ADD_LIBRARY(singleKukaParking
	SHARED
	#PACKAGE $PACKAGE$
	SOURCE
		src/singleKukaParking.C

	LINK_LIBS
		MIRABase
		MIRAFramework
)

###############################################################################

MIRA_ADD_LIBRARY(doubleKukaParking
	SHARED
	#PACKAGE $PACKAGE$
	SOURCE
		src/doubleKukaParking.C

	LINK_LIBS
		MIRABase
		MIRAFramework
)

###############################################################################

MIRA_ADD_LIBRARY(kukaRemoteController
	SHARED
	#PACKAGE $PACKAGE$
	SOURCE
		src/kukaRemoteController.C
        src/gamepad.cc
        src/gamepadEvent.cc

	LINK_LIBS
		MIRABase
		MIRAFramework
)
