from setuptools import setup, find_packages

setup(
    name="dora-openlong-udp",
    version="0.3.10",
    packages=find_packages(),
    install_requires=[
        "dora-rs >= 0.3.9",
        "scipy",
    ],
    entry_points={
        'console_scripts': [
            'dora-openlong-udp = dora_openlong_udp.udp:main',
            
        ],
    },
)