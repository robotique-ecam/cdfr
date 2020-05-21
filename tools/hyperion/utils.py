#!/usr/bin/env python3


"""Hyperion utils."""


from colorama import Back, Fore, Style


def print_ok(*args, **kwargs):
    """Print args and ANSI [OK] with **kwargs."""
    print(*args, f'[{Fore.GREEN + "OK" + Style.RESET_ALL}]', **kwargs)


def print_warn(*args, **kwargs):
    """Print args and ANSI [WARN] with **kwargs."""
    print(*args, f'[{Fore.YELLOW + "WARN" + Style.RESET_ALL}]', **kwargs)


def print_error(*args, **kwargs):
    """Print args and ANSI [ERROR] with **kwargs."""
    print(*args, f'[{Fore.RED + "ERROR" + Style.RESET_ALL}]', **kwargs)


def print_critical(*args, **kwargs):
    """Print args and ANSI [ERROR] with **kwargs."""
    print(*args, f'[{Fore.WHITE + Back.RED + "CRITICAL" + Style.RESET_ALL}]', **kwargs)


if __name__ == '__main__':
    print_ok('Hyperion started')
    print_warn('This is a warning')
    print_error('This is a failure')
    print_critical('In case of crash')
