Import("env")

import shutil
import os

def after_build(source, target, env):
    print("   > Start of build project extra script")
    
    # Obtenir le chemin du fichier .bin compilé
    firmware_path = str(target[0])
    if not os.path.exists(firmware_path):
        print(f"Error: Firmware not found at {firmware_path}")
        return
        
    # Créer le dossier de sortie s'il n'existe pas
    current_dir = os.getcwd()
    output_dir = os.path.join(current_dir, "bin")
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        print(f"Created output directory: {output_dir}")
    
    # Nettoyer et copier le firmware.bin
    bin_input = firmware_path
    bin_output = os.path.join(output_dir, "firmware.bin")
    if os.path.exists(bin_output):
        try:
            os.remove(bin_output)
            print("Cleaned up existing firmware.bin")
        except Exception as e:
            print(f"Warning: Could not remove existing firmware.bin: {e}")
    
    # Nettoyer et copier le firmware.elf
    elf_input = os.path.splitext(firmware_path)[0] + ".elf"
    elf_output = os.path.join(output_dir, "firmware.elf")
    if os.path.exists(elf_output):
        try:
            os.remove(elf_output)
            print("Cleaned up existing firmware.elf")
        except Exception as e:
            print(f"Warning: Could not remove existing firmware.elf: {e}")
    
    # Copier les nouveaux fichiers firmware
    try:
        shutil.copy2(bin_input, bin_output)
        print(f"Successfully copied new firmware.bin to: {bin_output}")
        
        shutil.copy2(elf_input, elf_output)
        print(f"Successfully copied new firmware.elf to: {elf_output}")
    except Exception as e:
        print(f"Error copying firmware files: {e}")
    
    print("   > End of build project extra script")

# Ajouter le script après la création du binaire
env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", after_build)
print("Extra script loaded")