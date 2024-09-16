import rospy
from std_msgs.msg import String

MONSTERS_A = ['Fire', 'Water', 'Earth']
MONSTERS_B = ['Rock', 'Thunder', 'Wind']
INITIAL_HP_A = [300, 400, 500]
INITIAL_HP_B = [300, 400, 500]

player_a_hp = {monster: INITIAL_HP_A[i] for i, monster in enumerate(MONSTERS_A)}
player_b_hp = {monster: INITIAL_HP_B[i] for i, monster in enumerate(MONSTERS_B)}

def player_a_callback(msg):
    moves = msg.data.split(',')
    print("Player A's moves:", moves)
    process_moves('A', moves)

def player_b_callback(msg):
    moves = msg.data.split(',')
    print("Player B's moves:", moves)
    process_moves('B', moves)

def updatedmonsterhitpoints():
    print("\nUpdated monster hitpoints:")
    print("Player A's monsters:", player_a_hp)
    print("Player B's monsters:", player_b_hp)

def process_moves(player, moves):
    attacker_hp = player_a_hp if player == 'A' else player_b_hp
    defender_hp = player_b_hp if player == 'A' else player_a_hp

    print("\nAttack moves:")
    for i, move in enumerate(moves):
        attacker = MONSTERS_A[i] if player == 'A' else MONSTERS_B[i]
        if move == '{1},{monster}':  # ATTACK ONE
            if monster == 1:
                if attacker == 'Fire':
                    defender = 'Rock'
                if attacker == 'Rock':
                    defender = 'Fire'
            elif monster == 2:
                if attacker == 'Water':
                    defender = 'Thunder'
                if attacker == 'Water':
                    defender = 'Thunder'
            elif monster == 3:
                if attacker == 'Earth':
                    defender = 'Wind'
                if attacker == 'Earth':
                    defender = 'Wind'


            if attacker == 'Fire' or 'Rock':
                damage = 0.1 * 300
            elif attacker == 'Water' or 'Thunder':
                damage = 0.1 * 400
            elif attacker == 'Earth' or 'Wind':
                damage = 0.1 * 500


            if defender in defender_hp:
                defender_hp[defender] -= damage
                print(f"{attacker}'s {attacker} attack deals {damage} damage to {defender}.")
            else:
                print(f"{defender} does not exist or has already been defeated.")
        elif move == '2':  # ATTACK ALL
            if attacker == 'Fire' or 'Rock':
                damage = 0.1 * 300
            elif attacker == 'Water' or 'Thunder':
                damage = 0.1 * 400
            elif attacker == 'Earth' or 'Wind':
                damage = 0.1 * 500
            #damage = 0.1 * 300 if attacker == 'Fire' elif 
            for monster in defender_hp:
                defender_hp[monster] -= damage
            print(f"{attacker}'s {attacker} attack deals {damage} damage to all monsters.")
        #pub.Pubish(updatedmonsterhitpoints)

    updatedmonsterhitpoints()

    if all(hp <= 0 for hp in defender_hp.values()):
        winner = 'Player A' if player == 'B' else 'Player B'
        print(f"\n{winner} wins!")
        rospy.signal_shutdown("Game Over")

def game_moderator():
    rospy.init_node('game_moderator')

    rospy.Subscriber('player_a_moves', String, player_a_callback)
    rospy.Subscriber('player_b_moves', String, player_b_callback)
    pub = rospy.Publisher('moderator', String, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    try:
        game_moderator()
    except rospy.ROSInterruptException:
        pass
