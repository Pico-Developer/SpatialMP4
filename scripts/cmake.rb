class Cmake < Formula
  desc "Cross-platform make"
  homepage "https://www.cmake.org/"
  # url "https://github.com/Kitware/CMake/releases/download/v3.22.1/cmake-3.22.1.tar.gz"
  # sha256 "0e998229549d7b3f368703d20e248e7ee1f853910d42704aa87918c213ea82c0"
  url "https://github.com/Kitware/CMake/releases/download/v3.24.1/cmake-3.24.1.tar.gz"
  sha256 "4931e277a4db1a805f13baa7013a7757a0cbfe5b7932882925c7061d9d1fa82b"
  license "BSD-3-Clause"
  head "https://gitlab.kitware.com/cmake/cmake.git"

  livecheck do
    url "https://cmake.org/download/"
    regex(/Latest Release \(v?(\d+(?:\.\d+)+)\)/i)
  end

  depends_on "sphinx-doc" => :build

  uses_from_macos "ncurses"

  on_linux do
    depends_on "openssl@1.1"
  end

  # The completions were removed because of problems with system bash

  # The `with-qt` GUI option was removed due to circular dependencies if
  # CMake is built with Qt support and Qt is built with MySQL support as MySQL uses CMake.
  # For the GUI application please instead use `brew install --cask cmake`.

  def install
    args = %W[
      --prefix=#{prefix}
      --no-system-libs
      --parallel=#{ENV.make_jobs}
      --datadir=/share/cmake
      --docdir=/share/doc/cmake
      --mandir=/share/man
      --sphinx-build=#{Formula["sphinx-doc"].opt_bin}/sphinx-build
      --sphinx-html
      --sphinx-man
    ]
    on_macos do
      args += %w[
        --system-zlib
        --system-bzip2
        --system-curl
      ]
    end

    system "./bootstrap", *args, "--", *std_cmake_args,
                                       "-DCMake_INSTALL_EMACS_DIR=#{elisp}"
    system "make"
    system "make", "install"
  end

  test do
    (testpath/"CMakeLists.txt").write("find_package(Ruby)")
    system bin/"cmake", "."
  end
end
