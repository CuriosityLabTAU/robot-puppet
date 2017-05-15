from naoqi import ALProxy
import sys
import time
import os


### use this for movments http://funlab.nd.edu/the-nao-base/movements-with-sound/salutations/
### recored behaviors


###
# tts = ALProxy("ALTextToSpeech", "192.168.0.100", 9559)
# tts.say("Hello, world!")

### run Behavior
managerProxy = ALProxy("ALBehaviorManager","192.168.0.101", 9559)
#managerProxy.post.runBehavior("movements/no_no")
#managerProxy.post.runBehavior("movements/raise_the_roof/raise_the_roof")
managerProxy.post.runBehavior("movements//introduction_all_0")
names = managerProxy.getInstalledBehaviors()
print "Behaviors on the robot:"
print names

audioProxy = ALProxy("ALAudioPlayer", "192.168.0.101", 9559)
#audioProxy.playFile('/home/nao/naoqi/wav/nih_howie/howie_wav/' + data.data + '.wav',1.0,0.0)
ssl = audioProxy.getInstalledSoundSetsList()
print ssl

#audioProxy.playFile('/home/nao/naoqi/wav/ask_again_0.wav',1.0,0.0)
#audioProxy.playFile('/home/nao/wav/introduction_all_0.wav',1.0,0.0)


#sounds-67efea/
#audioProxy.playFile('introduction_all_0.wav',1.0,0.0)
#audioProxy.playSoundSetFile("introduction_all_0.wav")

### Animated Speech for recored
AnimatedSpeechProxy = ALProxy("ALAnimatedSpeech","192.168.0.101", 9559)
#AnimatedSpeechProxy.say("Hello Look I can stop moving  you see ?",'contextual')






