from classes.Commands.RenderCommand import RenderCommand
from classes.basic.command import Command


class LayersRenderInvoker:
    _render = None

    def set_render(self, command: Command):
        self._render = command

    def render_map(self) -> None:
        self._render.execute()


if __name__ == '__main__':
    invoker = LayersRenderInvoker()
    invoker.set_render(RenderCommand())
