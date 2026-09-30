#!/bin/bash
# Source : ~/pca_wake.sh du Pi5 (hors git jusqu'au 2026-09-13), copie a l'identique ci-dessous.
# /!\ ECRIT DES CONSIGNES PWM : les 4 servos (axe 4, axe 5, verrou de tete, pince) VONT BOUGER
# vers ces valeurs. Feu vert de Sam requis, bras degage.
# Reveille le PCA9685 apres une coupure d'alim (sort du bit SLEEP, remet 50 Hz)
# puis reecrit les 4 canaux a leur valeur de consigne courante.
# CH0=409 (axe4 135deg)  CH1=388 (axe5 125.8deg)  CH2=216 (verrou 50deg)  CH3=352 (pince ouverte 110deg)
set -e
echo "AVANT : MODE1=$(i2cget -y 1 0x40 0x00)  PRESCALE=$(i2cget -y 1 0x40 0xFE)"
i2cset -y 1 0x40 0x00 0x10      # SLEEP (requis pour changer le prescale)
i2cset -y 1 0x40 0xFE 0x79      # prescale 121 -> 50 Hz
i2cset -y 1 0x40 0x00 0x20      # reveil + auto-increment
i2cset -y 1 0x40 0x00 0xA0      # RESTART
echo "APRES : MODE1=$(i2cget -y 1 0x40 0x00)  PRESCALE=$(i2cget -y 1 0x40 0xFE)"
w() { b=$((6+4*$1)); i2cset -y 1 0x40 $b 0x00; i2cset -y 1 0x40 $((b+1)) 0x00; i2cset -y 1 0x40 $((b+2)) $2; i2cset -y 1 0x40 $((b+3)) $3; }
w 0 0x99 0x01
w 1 0x84 0x01
w 2 0xD8 0x00
w 3 0x60 0x01
echo "--- canaux ---"
for ch in 0 1 2 3; do b=$((6+4*ch)); ol=$(i2cget -y 1 0x40 $((b+2))); oh=$(i2cget -y 1 0x40 $((b+3))); o=$(( ($(printf %d $oh)&0x0F)*256 + $(printf %d $ol) )); printf "   CH%s=%s ticks\n" $ch $o; done
