# -*- mode: python -*-
# Command:
#
# 	pyinstaller MainApp.spec
#   

block_cipher = None

a = Analysis(['MainApp.py'],
             pathex=['.'],
             binaries=[],
			 datas=[('gui/logo.ico','gui/logo.ico'), ('gui/logo.ico','.') ],
             hiddenimports=[],
             hookspath=[],
             hooksconfig={},
             runtime_hooks=[],
             excludes=[],
             win_no_prefer_redirects=False,
             win_private_assemblies=False,
             cipher=block_cipher,
             noarchive=False)
pyz = PYZ(a.pure, a.zipped_data,
             cipher=block_cipher)

exe = EXE(pyz,
          a.scripts,
          a.binaries,
          a.zipfiles,
          a.datas,  
          [],
          name='MainApp',
          debug=False,
          bootloader_ignore_signals=False,
          strip=False,
          upx=True,
          upx_exclude=[],
          runtime_tmpdir=None,
          console=True,
          disable_windowed_traceback=False,
          target_arch=None,
		  icon='gui/logo.ico',
          codesign_identity=None,
          entitlements_file=None )

app = BUNDLE(
	exe,
	name='MainApp.app',
	icon='gui/logo.ico',
	bundle_identifiers=None,
	info_plist={'NSHighResolutionCapable':'True'},
)

		  
