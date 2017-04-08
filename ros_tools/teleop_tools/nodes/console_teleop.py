#!/usr/bin/env python
# ==============================================================================
#
# dgerod@xyz-lab.org.es
# ==============================================================================

import roslib; roslib.load_manifest('teleop_tools')
import rospy;
from manual_pendant import TeachPendant, TargetConn

import urwid
from urwid_satext.files_management import FileDialog
from urwid_satext.sat_widgets import Menu, SurroundedText
from urwid_satext.sat_widgets import TabsContainer, NotificationBar, LabelLine

import time

# ------------------------------------------------------------------------------

class MyFileDialog(FileDialog):

    PALETTE = [('title', 'black', 'light gray', 'standout,underline'),
               ('default', 'default', 'default'),
               ('default_focus', 'default,bold', 'default'),
               ('directory', 'dark cyan, bold', 'default'),
               ('directory_focus', 'dark cyan, bold', 'dark green'),
               ('separator', 'brown', 'default'),
               ]

    def __init__(self, App):
        super(self.__class__, self).__init__(self.choiceOk, self.choiceCancel)
        self._App = App

    def choiceOk(self, filename):
        new_widget = urwid.Filler(urwid.Text(filename,align='center'))
        self._App.loop.widget = new_widget
        self._App.loop.draw_screen()        
        
        global _tp, _conn
        _tp._TrajectoryFile = filename
        _tp._Trajectory.loadPtpFromFile(filename, _conn._UserTopic)
      
        self._App.show()
    
    def choiceCancel(self, control):
        """This callback is called when user cancelled the dialog"""
        self._App.show()
    
class MainMenu(object):

    def __init__(self, App):
        self._App = App
        
    def build(self):
        self._Menu = Menu(self._App.loop)
        
        menu1 = "File"
        self._Menu.addMenu(menu1, "Load Trajectory", self.choiceLoadTrajectory)
        self._Menu.addMenu(menu1, "Exit (Ctrl+X)", self.choiceExit, 'ctrl x')
        menu2 = "Trajectory"
        self._Menu.addMenu(menu2, "Play", self.choicePlayTrajectory)
        self._Menu.addMenu(menu2, "Next", self.choiceNextPosition) 
        self._Menu.addMenu(menu2, "Previous", self.choicePrevPosition)
        self._Menu.addMenu(menu2, "Rewind", self.choiceRewindTrajectory)
        menu3 = "Help"
        self._Menu.addMenu(menu3, "ROS", self.menu_cb)
        self._Menu.addMenu(menu3, "About App", self.menu_cb)

        return self._Menu

    def choiceLoadTrajectory(self, menu_data):
        new_widget = MyFileDialog(self._App)
        self._App.loop.widget = new_widget  
        self._App.bottom_bar.addMessage("Trajectory loaded.")
        
    def choicePlayTrajectory(self, menu_data):
        self._App._showMessage("Executing trajectory.")
        self._App.bottom_bar.addMessage("Waiting.")
        
        global _tp
        _tp.playTrajectory()
        
        self._App.bottom_bar.addMessage("Ready.")
        
    def choiceNextPosition(self, menu_data):
        global _tp, _conn
        p = _tp._Trajectory.nextPosition()
        _conn.sendPosition(p)
        
    def choicePrevPosition(self, menu_data):
        global _tp, _conn
        p = _tp._Trajectory.previousPosition()
        _conn.sendPosition(p)
            
    def choiceRewindTrajectory(self, menu_data):
        global _tp
        _tp._Trajectory.rewind()

    def choiceExit(self, menu_data): 
        global _conn
        _conn.stop()
        self._App._exit()
        
    def menu_cb(self, menu_data):
        self._App._showMessage("Menu selected: %s/%s" % menu_data)
                
class TeachPendantApp(object):
        
    PALETTE = [('menubar', 'light gray,bold', 'dark red'),
               ('menubar_focus', 'light gray,bold', 'dark green'),
               ('menuitem', 'light gray,bold', 'dark red'),
               ('menuitem_focus', 'light gray,bold', 'dark green'),
               ('title', 'black', 'light gray', 'standout,underline'),
               ('default', 'default', 'default'),
               ('default_focus', 'default,bold', 'default'),
               ('directory', 'dark cyan, bold', 'default'),
               ('directory_focus', 'dark cyan, bold', 'dark green'),
               ('separator', 'brown', 'default'),
               ]
    
    def __init__(self):
        
        self.frame = urwid.Frame(urwid.Filler(urwid.Text('Menu demo', align='center')))
        self.loop = urwid.MainLoop(self.frame, TeachPendantApp.PALETTE, unhandled_input=self.keyHandler)

        self.menu = MainMenu(self)
        self.frame.set_header(self.menu.build())
        self.frame.set_focus('header')
        
        tabs = TabsContainer()
        tabs.addTab("Point-to-point", None, True)
        self.message_box = urwid.Text("")
        
        self.bottom_bar = NotificationBar()
        self.bottom_bar.addMessage(u"Ready.")
        
        pile = urwid.Pile([self.menu._Menu, 
                           #urwid.AttrWrap(SurroundedText(" Control "), 'bright'),
                           #tabs,
                           urwid.AttrWrap(SurroundedText(u" Message "), 'bright'),
                           self.message_box,
                           urwid.AttrWrap(SurroundedText(u" Status "), 'bright'),
                           self.bottom_bar])
        
        self.top = urwid.Filler(pile, valign='top')
        self.loop.widget = self.top
    
    def attach(self, TeachPendant):
        self.tp = TeachPendant
        
    def run(self):
        self.loop.run()

    def show(self):
        self.loop.widget = self.top
        
    def _showMessage(self, message):
        # new_widget = urwid.Filler(urwid.Text(message, align='center'))
        # self.loop.widget = new_widget
        # self.loop.draw_screen()
        # time.sleep(5)
        # self.loop.widget = self.top
        
        self.message_box.set_text(message)
        self.loop.draw_screen()        
        
    def _exit(self):
        new_widget = urwid.Filler(urwid.Text(exit("Exiting through 'Exit' menu item"), 
                                             align='center'))
        self.loop.widget = new_widget
        self.loop.draw_screen()
        time.sleep(1)
        raise urwid.ExitMainLoop()

    def keyHandler(self, input):
        """We leave if user press a quit char"""
        if input in ('esc','q','Q'):
            self.tp._Conn.stop()
            self._exit()
            #raise urwid.ExitMainLoop()
        else:
            return self.menu._Menu.checkShortcuts(input)
        
# ------------------------------------------------------------------------------

if __name__ == '__main__':

    try:
        rospy.init_node("teleop_pendant")    

        global _conn, _tp
        _conn = TargetConn()
        _tp = TeachPendant(_conn)
        #_tp = TeachPendant(_conn, "iri_wam")
    
        top = TeachPendantApp()
        top.attach(_tp)
        top.run()

    except rospy.ROSInterruptException: 
        pass

# ==============================================================================
