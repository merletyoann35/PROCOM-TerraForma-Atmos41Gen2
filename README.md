# PROCOM-TerraForma-Atmos41Gen2
Configuration Station Atmos 41 Gen 2 pour projet Terra Forma

- La documentation comporte les configurations des autres matériels utilisés durant la phase de test.

- Le fichier LoRaWAN_Final.ino est le fichier chargé sur la station qui fonctionne pour réaliser la récupération et le stockage des données de la station de bout en bout.

- Le fichier RecuperationMesureEnDirect.ino sert à récupérer avec un lien direct (USB) sur la station météo, toutes les données partagées par la station.

- Le fichier collect_send_CoAP.ino est la première version fonctionnelle de la version finale du projet avec le protocole COAP d'intégré.


### Points non précisés dans la documentation :
<p style="margin-left: 2em;">
  Les fichiers LoRaWAN_Final.ino et collect_send_CoAP.ino, ne transmettent que 6 mesures des 17 possiblement récupérables sur la station Atmos.<br>
</p>
<p style="margin-left: 2em;">
  Les mesures manquantes peuvent être rajoutées à la suite de celles déjà envoyées.<br>
</p>
<p style="margin-left: 2em;">
  De plus, certaines mesures supplémentaires peuvent être rajoutées par la suite. Dans le manuel de la station on peut retrouver à partir de la page 19 des calculs permettant d'avoir de nouvelles mesures en fonction des autres mesures récoltées sur la station (https://publications.metergroup.com/Manuals/20937_ATMOS41_Gen2_Manual_Web.pdf?_gl=1*fwnx8g*_gcl_au*MzcwNjU3NzU1LjE3NzQ5NDgwMTU.)<br>
</p>

