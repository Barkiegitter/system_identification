from setuptools import setup

setup(
    name='control',
    version='0.1.0',
    description="Controller module",
    author='Maarten de Vries',
    author_email='maarten@captainai.com',
    packages=['control'],
    install_requries=['numpy', 'cap-comm'],
    scripts=[
        'utilities/filters'
        'utilities/pid_corrections'
    ]
)