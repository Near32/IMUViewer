#ifndef TYPECOMMAND_H
#define TYPECOMMAND_H

enum TypeCommand {
	TCLoadMap,
	TCNew,
	TCSave,
	TCLoad,
	TCPause,
	TCGoOn,
	TCQuit,
	TCRigidBody,
	TCCamera,
	TCCameraOnMouseButton,
	TCCameraOnMouseMotion,
//	TCCameraOnMouseWheel,
	TCCameraOnKeyboard,
	
	//DEBUGGING :
	TCSimulateStride
};

#endif
