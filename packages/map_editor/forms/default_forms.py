from PyQt5.QtWidgets import QMessageBox


def question_form_yes_no(parent, title, text):
    box = QMessageBox(parent)
    box.setIcon(QMessageBox.Question)
    box.setWindowTitle(title)
    box.setText(text)
    box.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
    box.setDefaultButton(QMessageBox.Yes)
    return box.exec()


def form_yes(parent, title, text):
    box = QMessageBox(parent)
    box.setIcon(QMessageBox.Information)
    box.setWindowTitle(title)
    box.setText(text)
    box.setStandardButtons(QMessageBox.Yes)
    box.setDefaultButton(QMessageBox.Yes)
    return box.exec()
