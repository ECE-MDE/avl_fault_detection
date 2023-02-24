import rospy

def main():
    rospy.init_node('listener', anonymous=True)
    print("\n\n\n\n\n\n\n")
    for name, type in rospy.get_published_topics():
        print(f"{name} {type}")
    rospy.spin()
    ...

if __name__ == '__main__':
    main()