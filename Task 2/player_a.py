import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id())

def player_a():
    rospy.init_node('player_a', anonymous=True)

    rospy.sleep(1)  # Wait for server to initialize

    pub = rospy.Publisher('player_a_moves', String, queue_size=1)
    rospy.Subscriber('moderator', String, callback)

    while not rospy.is_shutdown():
        fire_move = int(input("Fire's Turn (1 for Attack One, 2 for Attack All): "))
        if fire_move == 1:
            monsterattack = int(input("Enter the name of monster to attack (1 for Rock, 2 for Thunder and 3 for wind): "))
            move = f"{fire_move},{monsterattack}"
            pub.publish(move)
        elif fire_move == 2:
            move = f"{fire_move}"
            #attack = input("Enter the opponent's monster to attack: ")
            #move += f",{attack}"
            pub.publish(move)
        else:
            print("Invalid move. Please try again.")
            continue

        water_move = int(input("Water's Turn (1 for Attack One, 2 for Attack All): "))
        if water_move == 1:
            monsterattack = int(input("Enter the name of monster to attack (1 for Rock, 2 for Thunder and 3 for wind): "))
            move = f"{water_move},{monsterattack}"
            pub.publish(move)
        elif water_move == 2:
            move = f"{water_move}"
            #attack = input("Enter the opponent's monster to attack: ")
            #move += f",{attack}"
            pub.publish(move)
        else:
            print("Invalid move. Please try again.")
            continue

        earth_move = int(input("Earth's Turn (1 for Attack One, 2 for Attack All): "))
        if earth_move == 1:
            monsterattack = int(input("Enter the name of monster to attack (1 for Rock, 2 for Thunder and 3 for wind): "))
            move = f"{earth_move},{monsterattack}"
            pub.publish(move)
        elif earth_move == 2:
            move = f"{earth_move}"
            #attack = input("Enter the opponent's monster to attack: ")
            #move += f",{attack}"
            pub.publish(move)
        else:
            print("Invalid move. Please try again.")
            continue

if __name__ == '__main__':
    player_a()
