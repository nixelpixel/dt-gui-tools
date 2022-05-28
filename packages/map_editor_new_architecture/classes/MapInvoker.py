from classes.basic.command import Command


class MapInvoker:
    """
    EN: The MapInvoker is associated with one or several commands. It sends a
    request to the command.
    RU: Отправитель связан с одной или несколькими командами. Он отправляет
    запрос команде.
    """

    _on_start = None
    _on_finish = None

    """
    EN: Initialize commands.
    RU: Инициализация команд.
    """

    def set_on_start(self, command: Command):
        self._on_start = command

    def set_on_finish(self, command: Command):
        self._on_finish = command

    def render(self) -> None:
        """
        EN: The MapInvoker does not depend on concrete command or receiver classes.
        The MapInvoker passes a request to a receiver indirectly, by executing a
        command.
        RU: Отправитель не зависит от классов конкретных команд и получателей.
        Отправитель передаёт запрос получателю косвенно, выполняя команду.
        """

        print("MapInvoker: Does anybody want something done before I begin?")
        if isinstance(self._on_start, Command):
            self._on_start.execute()

        print("MapInvoker: ...doing something really important...")

        print("MapInvoker: Does anybody want something done after I finish?")
        if isinstance(self._on_finish, Command):
            self._on_finish.execute()
