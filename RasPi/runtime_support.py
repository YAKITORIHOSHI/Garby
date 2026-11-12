"""Process ownership helpers for GARBY; no ROS, BLE, or Firebase imports."""
import os
from pathlib import Path
import stat
import tempfile


class SingleInstanceGuard:
    """Hold an OS lock until close/process exit; never unlink the lock inode."""
    def __init__(self, path=None):
        default = Path("/tmp" if os.name == "posix" else tempfile.gettempdir()) / "garby-bridge.lock"
        self.path = Path(path or os.environ.get("GARBY_RUNTIME_LOCK_FILE") or default)
        self.fd = None

    def __enter__(self):
        flags = os.O_RDONLY if os.name == "posix" else os.O_RDWR
        flags |= getattr(os, "O_NOFOLLOW", 0)
        try:
            try:
                self.fd = os.open(self.path, flags)
            except FileNotFoundError:
                try:
                    self.fd = os.open(self.path, flags | os.O_CREAT | os.O_EXCL, 0o644)
                    if os.name == "posix":
                        os.fchmod(self.fd, 0o644)  # Contains no data; other service users may lock it.
                except FileExistsError:
                    self.fd = os.open(self.path, flags)
            if not stat.S_ISREG(os.fstat(self.fd).st_mode):
                raise RuntimeError(f"Runtime lock is not a regular file: {self.path}")
            if os.name == "posix":
                import fcntl
                fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            else:
                import msvcrt
                if os.fstat(self.fd).st_size == 0:
                    os.write(self.fd, b"0")
                os.lseek(self.fd, 0, os.SEEK_SET)
                msvcrt.locking(self.fd, msvcrt.LK_NBLCK, 1)
        except OSError as error:
            self.close()
            raise RuntimeError(
                f"GARBY runtime lock unavailable ({self.path}); stop another bridge instance "
                f"or correct lock-file access: {error}") from error
        except Exception:
            self.close()
            raise
        return self

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None

    def __exit__(self, *args):
        self.close()
