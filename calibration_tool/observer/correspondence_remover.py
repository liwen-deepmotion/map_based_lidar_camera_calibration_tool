from observer.base_observer import BaseObserver
from observer.event.events import CustomEvent


class CorrespondenceRemover(BaseObserver):

    def __init__(self, editor: 'MapBasedCalibrator'):
        super().__init__(editor)

        self.QT_SIGNAL_CALLBACK_TUPLES = [
            (self.editor.side_bar_widget.clear_correspondence_btn.clicked,
             self.clear_last_correspondence)
        ]

    def clear_last_correspondence(self):
        if self.editor.layer_manager.correspondence_layer(False) is None:
            return
        self.editor.layer_manager.correspondence_layer().remove_correspondence(
            self.editor.layer_manager.correspondence_layer().correspondences()[-1])
        
        self.update()

    def clear_all_correspondences(self):
        if self.editor.layer_manager.correspondence_layer(False) is None:
            return

        self.editor.layer_manager.correspondence_layer().clear()

        self.update()
