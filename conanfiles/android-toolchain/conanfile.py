from conans import ConanFile, tools
import os

class AndroidToolchainConan(ConanFile):
    name = "android-toolchain"
    version = "r20"
    settings = "os_build"
    description = "Conan package for android toolchain."
    url = "https://developer.android.com/ndk"
    license = "Apache License 2.0"

    def source(self):
        tools.get("https://dl.google.com/android/repository/android-ndk-{}-linux-x86_64.zip".format(self.version), keep_permissions=True)

    def package(self):
        self.copy("*")

    def package_info(self):
        self.env_info.ANDROID_NDK_HOME = os.path.join(self.package_folder, "android-ndk-{}".format(self.version))
        self.env_info.ANDROID_TOOLCHAIN_FILE_PATH = os.path.join(self.package_folder, "android-ndk-{}".format(self.version), "build/cmake/android.toolchain.cmake")
