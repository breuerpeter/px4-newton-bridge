"""Generate gRPC/protobuf Python stubs from proto/newton_api.proto."""

import subprocess
import sys
from pathlib import Path


def main():
    root = Path(__file__).resolve().parent.parent
    proto_dir = root / "proto"
    out_dir = root / "newton_api" / "generated"
    out_dir.mkdir(exist_ok=True)

    cmd = [
        sys.executable, "-m", "grpc_tools.protoc",
        f"--proto_path={proto_dir}",
        f"--python_out={out_dir}",
        f"--grpc_python_out={out_dir}",
        str(proto_dir / "newton_api.proto"),
    ]
    print(" ".join(cmd))
    subprocess.check_call(cmd)

    # Fix imports: protoc generates bare `import newton_api_pb2` but we need
    # relative imports since stubs live inside a package.
    grpc_file = out_dir / "newton_api_pb2_grpc.py"
    text = grpc_file.read_text()
    text = text.replace("import newton_api_pb2 as", "from . import newton_api_pb2 as")
    grpc_file.write_text(text)

    print(f"Generated stubs in {out_dir}")


if __name__ == "__main__":
    main()
