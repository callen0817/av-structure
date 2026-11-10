# -*- mode: python ; coding: utf-8 -*-
import os
import sys
from PyInstaller.utils.hooks import collect_all

# Collect all resources for open3d
datas_o3d, binaries_o3d, hiddenimports_o3d = collect_all('open3d')

block_cipher = None

# We need to manually add files/folders if needed
# The script depends on:
# - io_modules/
# - simulation/
# - stations/
# - config.py
# - virtual_scanner.py

a = Analysis(
    ['gui.py'],
    pathex=[],
    binaries=binaries_o3d,
    datas=datas_o3d + [
        ('config.py', '.'),
        ('virtual_scanner.py', '.'),
        ('io_modules', 'io_modules'),
        ('simulation', 'simulation'),
        ('stations', 'stations'),
    ],
    hiddenimports=hiddenimports_o3d + [
        'customtkinter',
        'pye57',
        'laspy',
        'lazrs',
        'pyquaternion',
        'scipy.spatial.transform._rotation_groups', # Common missing for scipy
    ],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    win_no_prefer_redirects=False,
    win_private_assemblies=False,
    cipher=block_cipher,
    noarchive=False,
)

pyz = PYZ(a.pure, a.zipped_data, cipher=block_cipher)

exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name='AV_Structure_Converter',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    console=False, # Set to False for GUI app
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)

coll = COLLECT(
    exe,
    a.binaries,
    a.zipfiles,
    a.datas,
    strip=False,
    upx=True,
    upx_exclude=[],
    name='AV_Structure_Converter',
)
