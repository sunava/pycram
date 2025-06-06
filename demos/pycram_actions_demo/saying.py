
import sys

if __name__ == '__main__':
    import rospy
    argv = rospy.myargv()
    if len(argv) > 1 and argv[1] == '--help':
        print('Usage: %s \'String to say.\'' % argv[0])
        print('       %s < file_to_say.txt' % argv[0])
        print()
        print('Says a string. For a string on the command line, you must use quotes as')
        print('appropriate. For a string on standard input, the command will wait for')
        print('EOF before saying anything.')
        exit(-1)

    # Import after printing usage for speed.
    from sound_play.msg import SoundRequest
    from sound_play.libsoundplay import SoundClient

    if len(argv) == 1:
        print('Awaiting something to say on standard input.')

    # Ordered this way to minimize wait time.
    rospy.init_node('say', anonymous=True)
    soundhandle = SoundClient()
    rospy.sleep(1)

    voice = ('voice_us2_mbrola')
    volume = 100.0

    if len(argv) == 1:
        s = sys.stdin.read()
    else:
        s = argv[1]

        if len(argv) > 2:
            voice = argv[2]
        if len(argv) > 3:
            volume = float(argv[3])

    rospy.loginfo('Saying: %s' % s)
    rospy.loginfo('Voice: %s' % voice)
    rospy.loginfo('Volume: %s' % volume)

    soundhandle.say(s, voice, volume)
    rospy.sleep(1)
