"""支持使用 ``python -m lissajous_vision`` 启动命令行程序。"""

from .cli import main


if __name__ == "__main__":
    # 把 CLI 返回码交给操作系统，便于批处理判断成功、无效结果或输入错误。
    raise SystemExit(main())
