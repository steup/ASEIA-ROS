from qt_gui.plugin import Plugin
from python_qt_binding.QtWidgets import QWidget

class ChannelsGUI(Plugin):

    def __init__(self, context):
        super(ChannelsGUI, self).__init__(context)
        self.setObjectName('Channels')
        self._widget = QWidget()
        self._widget.setObjectName('MyPluginUi')
        self._widget.setWindowTitle('ASEIA Channels')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
