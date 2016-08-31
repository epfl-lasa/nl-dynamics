[![Build Status](https://travis-ci.com/epfl-lasa/nl-dynamics.svg?token=BqUQb763tsVV4QyzLgBy&branch=master)](https://travis-ci.com/epfl-lasa/nl-dynamics)

This was a research project into combining natural language instructions with robot controllers.

Essentially, it enables you to use corrective kinesthetic demonstrations (on the robot) along with a natural language description to build a model at training time. This model can be used at execution time to update the controller online, using the same NL utterance.

This repository is unlikely to be useful to anyone.

There's some code in `demonstrations` that segments the corrective part of a demonstration from a complete trajectory, using the original dynamics. This was developed by Stephane Ballmer during a semester project.

There's also some code in `interactive_demonstrations` that implements a state-machine-based dialogue interface. It asks questions, waits for responses, and has branches. It's implemented in the SMACH architecture, using pocketsphinx as the input. This was developed by Cyril Schmit during a semester project.
