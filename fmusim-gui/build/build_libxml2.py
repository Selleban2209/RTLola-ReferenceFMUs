from pathlib import Path
from subprocess import check_call
from fmpy.util import download_file
from fmpy import extract


archive = download_file('https://github.com/GNOME/libxml2/archive/refs/tags/v2.11.5.zip',
                        checksum='711675470075cc85ba450f56aff7424f1ecdef00bc5d1d5dced3ffecd1a9b772')

root = Path(__file__).parent

extract(archive, root)

build_dir = root / 'libxml2-x86_64-windows' / 'build'

install_prefix = root / 'libxml2-x86_64-windows' / 'install'

cmake_args = [
    '-G', 'Visual Studio 16 2019',
    '-A', 'x64'
]

check_call(
    ['cmake'] +
    cmake_args +
    ['-B', build_dir,
    '-D', f'CMAKE_INSTALL_PREFIX={install_prefix}',
    # '-D', 'BUILD_SHARED_LIBS=OFF',
    '-D', 'LIBXML2_WITH_ICONV=OFF',
    '-D', 'LIBXML2_WITH_LZMA=OFF',
    '-D', 'LIBXML2_WITH_PYTHON=OFF',
    '-D', 'LIBXML2_WITH_ZLIB=OFF',
    '-D', 'LIBXML2_WITH_TESTS=OFF',
    str(root / 'libxml2-2.11.5')]
)

check_call([
    'cmake',
    '--build', str(build_dir),
    '--config', 'Release',
    '--target', 'install'
])
