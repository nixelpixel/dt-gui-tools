from PyQt5 import QtWidgets, QtCore, QtGui


class ImageObject(QtWidgets.QLabel):
    """Base object class"""
    def __init__(self, img_path: str, parent: QtWidgets.QWidget, object_name: str = "", scale: tuple = (30, 60)):
        super(ImageObject, self).__init__()
        self.setParent(parent)
        self.name = object_name
        self.scaled_size = scale
        pixmap = QtGui.QPixmap(img_path)
        resize = QtCore.QSize(self.scaled_size[0], self.scaled_size[1])
        pixmap = pixmap.scaled(resize, aspectRatioMode=QtCore.Qt.KeepAspectRatio,
                                     transformMode=QtCore.Qt.SmoothTransformation)
        self.pixmap = pixmap
        self.setFixedSize(pixmap.width(), pixmap.height())
        self.setPixmap(self.pixmap)

    def rotate_object(self, angle_clockwise: int):
        if not angle_clockwise % 180 == 0:
            self.setFixedSize(self.pixmap.height(), self.pixmap.width())
        new_transform = QtGui.QTransform()
        new_transform.rotate(angle_clockwise)
        self.pixmap = self.pixmap.transformed(new_transform, QtCore.Qt.SmoothTransformation)
        self.setPixmap(self.pixmap)

    def move_object(self, new_position: tuple):
        self.move(QtCore.QPoint(new_position[0], new_position[1]))

    def change_position(self, new_position: tuple):
        self.move_object(new_position)
        self.pos().setX(new_position[0])
        self.pos().setY(new_position[1])


class DraggableImage(ImageObject):
    """Objects draggable class
        working with Qt coordinates
    """

    def __init__(self, img_path: str, parent: QtWidgets.QWidget, object_name: str = "", scale: tuple = (30, 60)):
        super(DraggableImage, self).__init__(img_path, parent, object_name, scale)
        self.drag_start_pos = None

    def mousePressEvent(self, event: QtGui.QMouseEvent):
        if event.button() == QtCore.Qt.LeftButton:
            self.setCursor(QtCore.Qt.ClosedHandCursor)
            self.drag_start_pos = event.pos()
            self.raise_()
        elif event.button() == QtCore.Qt.RightButton:
            self.rotate_object(90)
        super(DraggableImage, self).mousePressEvent(event)

    def mouseMoveEvent(self, event: QtGui.QMouseEvent):
        if self.drag_start_pos is not None:
            new_pos = self.pos() + event.pos() - self.drag_start_pos
            self.move_object((new_pos.x(), new_pos.y()))
        super(DraggableImage, self).mousePressEvent(event)

    def mouseReleaseEvent(self, event: QtGui.QMouseEvent):
        if event.button() == QtCore.Qt.LeftButton:
            self.setCursor(QtCore.Qt.ArrowCursor)
            new_pos = self.pos() + event.pos() - self.drag_start_pos
            self.change_position((new_pos.x(), new_pos.y()))
            self.drag_start_pos = None
        super(DraggableImage, self).mousePressEvent(event)


if __name__ == '__main__':
    import sys
    app = QtWidgets.QApplication(sys.argv)
    win = QtWidgets.QWidget()
    win.mainLayout = QtWidgets.QHBoxLayout()
    win.setLayout(win.mainLayout)
    win.setFixedSize(1000, 1000)
    win.img_list = [
        "../img/objects/watchtowers.png", "../img/objects/watchtowers.png"
    ]
    for img in win.img_list:
        draggable_image = DraggableImage(img, win)
        win.mainLayout.addWidget(draggable_image)
    win.show()
    sys.exit(app.exec_())
