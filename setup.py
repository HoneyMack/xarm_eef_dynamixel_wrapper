import setuptools
import pathlib


with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="xarm-eef-dynamixel-wrapper",
    version="0.0.1",
    install_requires=pathlib.Path("requirements.txt").read_text().splitlines(),
    author="Takuya Okubo",
    description="A package to control Dynamixel servos attached to RS485 of xArm/Lite6 end effector",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/HoneyMack/xarm_eef_dynamixel_wrapper",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires=">=3",
)
