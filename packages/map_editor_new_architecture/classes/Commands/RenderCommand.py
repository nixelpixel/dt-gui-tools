from classes.DtMapCommand import DtMapCommand


class RenderCommand(DtMapCommand):

    def execute(self) -> None:
        print(self.dm)
        print("!"*100)
