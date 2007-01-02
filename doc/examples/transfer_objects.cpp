// make changes to the objects you want to render
hd->clearEffects();
hd->addEffect( new HapticForceField );
hd->addEffect( new HapticSpring );

// nothing has changed on what is rendered yet, call transferObjects 
// to transfer the changes to the haptics rendering thread.
hd->transferObjects();
