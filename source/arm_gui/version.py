# -*- coding: utf-8 -*-
"""Application identity and version information.

Keep release numbers in this module so the running GUI and the Windows
executable resource always report the same version.
"""

APP_VERSION = '1.2'
WINDOWS_VERSION = (1, 2, 0, 0)
WINDOWS_VERSION_STRING = '.'.join(str(part) for part in WINDOWS_VERSION)

APP_NAME = 'ArmGUI'
PRODUCT_NAME = 'Daran GUI 4DOF'
FILE_DESCRIPTION = 'Daran 4DOF Robot Arm GUI'
COMPANY_NAME = 'Daran'
ORIGINAL_FILENAME = 'ArmGUI.exe'
INTERNAL_NAME = 'ArmGUI'
LEGAL_COPYRIGHT = 'Copyright (c) 2026 Daran'

# Conventional package-level spelling used by tooling and diagnostics.
__version__ = APP_VERSION


WINDOWS_STRING_FILE_INFO = {
    'CompanyName': COMPANY_NAME,
    'FileDescription': FILE_DESCRIPTION,
    'FileVersion': WINDOWS_VERSION_STRING,
    'InternalName': INTERNAL_NAME,
    'LegalCopyright': LEGAL_COPYRIGHT,
    'OriginalFilename': ORIGINAL_FILENAME,
    'ProductName': PRODUCT_NAME,
    'ProductVersion': WINDOWS_VERSION_STRING,
}
