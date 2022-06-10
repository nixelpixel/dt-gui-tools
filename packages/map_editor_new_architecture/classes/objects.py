from PyQt5 import QtWidgets, QtCore, QtGui


class ImageObject(QtWidgets.QLabel):
    """Base object class"""
    def __init__(self, img_path: str, parent: QtWidgets.QWidget, scale: tuple=(30,60)):
        super(ImageObject, self).__init__()
        self.setParent(parent)
        self.scaled_size = scale
        pixel_map = QtGui.QPixmap(img_path)
        resize = QtCore.QSize(self.scaled_size[0], self.scaled_size[1])
        pixel_map = pixel_map.scaled(resize, aspectRatioMode=QtCore.Qt.KeepAspectRatio,
                                     transformMode=QtCore.Qt.SmoothTransformation)
        self.setFixedSize(pixel_map.width(), pixel_map.height())
        self.setPixmap(pixel_map)


class DraggableImage(ImageObject):
    """Objects draggable class"""
    def __init__(self, img_path: str, parent: QtWidgets.QWidget, scale: tuple=(30,60)):
        super(DraggableImage, self).__init__(img_path, parent, scale)
        self.drag_start_pos = None

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self.setCursor(QtCore.Qt.ClosedHandCursor)
            self.drag_start_pos = event.pos()
            self.raise_()

    def mouseMoveEvent(self, event):
        if self.drag_start_pos is not None:
            self.move(self.pos() + event.pos() - self.drag_start_pos)

    def mouseReleaseEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self.setCursor(QtCore.Qt.ArrowCursor)
            self.pos().setX((self.pos() + event.pos() - self.drag_start_pos).x())
            self.pos().setY((self.pos() + event.pos() - self.drag_start_pos).y())
            self.drag_start_pos = None


if __name__ == '__main__':
    import sys
    app = QtWidgets.QApplication(sys.argv)
    win = QtWidgets.QWidget()
    win.mainLayout = QtWidgets.QHBoxLayout()
    win.setLayout(win.mainLayout)
    win.setFixedSize(1000, 1000)
    win.img_list = [
        "../img/objects/watchtower.png", "../img/objects/watchtower.png"
    ]
    for img in win.img_list:
        draggable_image = DraggableImage(img_path=img, parent=win)
        win.mainLayout.addWidget(draggable_image)
    win.show()
    sys.exit(app.exec_())
