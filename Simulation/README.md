# 🛰️ Planification de trajectoire pour drone avec SPSO + Simulation AirSim + Plugin Cesium

## 📌 Objectif du projet
Ce projet a pour objectif de développer un système de **planification de trajectoire optimisée** pour un drone en utilisant l'algorithme **SPSO (Simplified Particle Swarm Optimization)** basé sur des **coordonnées GPS**. Le trajet sera ensuite **simulé dans l’environnement AirSim** avec une cartographie réaliste grâce au **plugin Cesium**.



## Caractéristiques

- 🗺️ **Traitement de terrains réels à partir de fichiers GeoTIFF**
- 🏙️ **Détection des bâtiments depuis OpenStreetMap comme obstacles**
- 🔄 **Conversion de coordonnées sphériques à cartésiennes**
- 🧭 **Optimisation multi-objectifs du trajet (distance, sécurité, altitude)**
- ✈️ **Intégration directe avec AirSim/Unreal Engine**
- 📊 **Visualisation 3D des chemins optimaux**


## ⚙️ Technologies utilisées

| **Composant**         | **Détail**                                            |
| ----------------      | ----------------------------------------------------  |
| **🐍 Python**         | Implémentation de l'algorithme SPSO                   |
| **🧠 SPSO**           | Optimisation de trajectoire                           |
| **📍 GPS**             | Points de départ, cibles, obstacles                   |
| **🕹️ AirSim**         | Simulation du vol de drone                            |
| **🌍 Cesium Plugin**  | Environnement 3D réaliste avec données géographiques  |


## 🧠 1. Algorithme SPSO (Simplified Particle Swarm Optimization)

**Objectif** : Trouver un chemin optimal (le plus court, évitant les obstacles) entre deux points GPS.

**Références fiables** 

 - **SPSO** : [Clerc, M. (2006). Standard PSO 2006](https://hal.science/hal-01351529/document)

 - **GPS path planning using PSO** : [IEEE Paper Example](https://ieeexplore.ieee.org/document/6930552)

## 🛠️ 2. Simulation avec AirSim

- **AirSim** est un simulateur open-source développé par Microsoft pour les véhicules autonomes.
- Interface Python permettant de charger des trajectoires GPS et simuler le mouvement du drone.
- Contrôle possible via airsim.MultirotorClient.

**Installation et guides** :
- **Epic Games Launcher** : https://store.epicgames.com/fr/
- **Unreal Engine 4.27.x** : Télècharger depuis Epic Games Launcher 
- **AirSim** : https://microsoft.github.io/AirSim/build_windows/
- **Guide AirSIm** : https://microsoft.github.io/AirSim/ 

## 🌍 3. Environnement réaliste avec Cesium
- Le plugin Cesium for Unreal permet d’ajouter un environnement global 3D basé sur des données géographiques réelles.
- Synchronisation entre les coordonnées GPS générées par SPSO et le monde virtuel dans AirSim.

**Installation et guides** :
- **Cesium for Unreal** : https://github.com/CesiumGS/cesium-unreal/releases (Chercher version 4.27)
- **Guide Cesium** : https://cesium.com/learn/unreal/unreal-quickstart/


## Utilisation
- **Python Packages**

```bash
pip install numpy matplotlib rasterio osmnx geopandas shapely airsim scipy
```

- **Initialisation de votre fichier Settings.JSON et votre OriginGeopoint** : https://microsoft.github.io/AirSim/settings/
- **Dans votre projet UE4.27 avec le plugin Cesium choisir votre zone et votre OriginGeoPoint avec CesiumGeoreferance** : https://cesium.com/learn/unreal/unreal-quickstart/ .
- **Configurer votre terrain modele dans CreateModel.py** : Téléchargez un GeoTIFF de votre terrain depuis : https://geoprocessing.online/tool/srtm-dem-download/ .
- **Entrer votre point de départ et d'arrivée dans CreateModel.py** : (point de départ doit étre identique a votre OriginGeoPoint dans Settings.json et Geoteferance dans Cesium) .
- **Entrer votre coordonnee GPS pour générer les obstacles dans votre zone avec un rayon précis dans CreatModel.py** : Utiliser OSM.py pour la vérification .
- **Run the optimization**.

```bash
python SPSO_Main.py
```

## Personnalisation

**Paramètres ajustables**

- Taille de population et nombre d'itérations du PSO

- Pondérations de la fonction de coût

- Contraintes de vol du drone

- Rayon de détection des menaces

**Limitations**

- Modélisation simplifiée des bâtiments en cylindres

- Calculs de distance en degrés (approximatifs)

- Impact sur les performances avec de nombreuses menaces

**Améliorations Futures**

- Modèles 3D précis des bâtiments

- Replanification en temps réel

- Réalisation matériel