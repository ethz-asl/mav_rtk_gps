#!/usr/bin/env python  

def get_full_namespace(name_space, topic):
    # check if topic is already absolute
    if topic[0] == '/':
        full_name = topic
    else:
        full_name = name_space + topic

    return full_name