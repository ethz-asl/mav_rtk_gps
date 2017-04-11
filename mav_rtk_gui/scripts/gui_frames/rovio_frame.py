import rospy

from Tkinter import *
import std_srvs.srv

class RovioFrame:
    def __init__(self, parent_window):
        # Service Names.
        self.service_names = self.get_service_names()

        self.main_label = Label(parent_window, text="Reset Rovio (NPOSE = 0)", font="Times 14 bold")
        self.main_label.grid(row=0, columnspan=2)

        # Init rovio button.
        current_row = 1
        self.init_rovio_button = Button(parent_window,
                                        text = 'Reset Rovio',
                                        command = self.init_rovio_button_handler)

        self.init_rovio_button.grid(row = current_row, columnspan = 2)

        # Init rovio service message.
        current_row = 1
        self.init_message_label = Label(parent_window, text ='', wraplength = 450)
        self.init_message_label.grid(row = current_row, columnspan = 2)

        # Init rovio service client.
        self.init_rovio_srv = rospy.ServiceProxy(self.service_names['init_rovio'],
                                                 std_srvs.srv.Trigger)

    def get_service_names(self):
        # Rovio services.
        service_names = {}

        service_names['init_rovio'] = rospy.get_param('~init_rovio_srv',
                                                      'init_rovio_enu/send_reset_to_rovio')

        # Check if we should add a leading namespace
        if rospy.has_param('~namespace'):
            name_space = rospy.get_param('~namespace')
            name_space = '/' + name_space + '/'

            for key, value in service_names.iteritems():
                service_names[key] = helpers.get_full_namespace(name_space, value)

        return service_names

    def init_rovio_button_handler(self):
        try:
            response = self.init_rovio_srv()
            self.init_message_label['text'] = response.message

        except rospy.ServiceException, e:
            message = 'Service call to reset Rovio internal state failed: %s' % e
            self.init_message_label['text'] = message
