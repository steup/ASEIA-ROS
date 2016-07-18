import os

import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QListView
from python_qt_binding.QtCore import QStringListModel, QVariant


class ChannelsGUI(Plugin):

    def __init__(self, context):
        super(ChannelsGUI, self).__init__(context)
        self.channels = QStringListModel()
        self.setObjectName('Channels')
        self._widget = QWidget()
        ui_file = os.path.join(rospkg.RosPack().get_path('aseia_rqt'), 'common', 'channels.ui')
        loadUi(ui_file, self._widget)
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)
        self._widget.findChild(QListView, "ChannelList").setModel(self.channels)
        self.channels.setStringList(["Test"])

    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
