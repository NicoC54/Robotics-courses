Voici votre Fiche Résumé GNC dans sa forme véritablement finale et intégrale. J'ai nettoyé les petits bugs d'affichage de votre copier-coller, restauré absolument toutes les analogies (la balançoire, la mélasse, le Point Bleu / Pin Rouge), gardé toutes les équations mathématiques détaillées, et surtout, **j'ai intégré votre brillante "Règle d'or de l'extraction" au tout début de l'Étape 2**, là où elle prend tout son sens.

C'est un document de niveau ingénieur complet.

---

# 🚀 Fiche Résumé GNC : Du Moteur au Filtre de Kalman (Version Intégrale)

## 1. La Physique et le Contrôle (Mixer d'un Drone)

Le contrôleur PID ne raisonne pas en nombre de moteurs, mais en **physique des solides**. Il génère des commandes abstraites que la matrice de mixage traduit pour le châssis physique.

* **Lien PID / Physique :** Erreur de vitesse linéaire $\rightarrow$ Accélération linéaire $\rightarrow$ **Force** (Thrust sur Z). Erreur de vitesse angulaire $\rightarrow$ Accélération angulaire $\rightarrow$ **Couple/Torque** (Roll, Pitch, Yaw).
* **La Matrice de Mixage (+1 / -1) :**
* *Roulis/Tangage* : Se déduit par l'effet de levier (on accélère d'un côté, on ralentit l'autre).
* *Lacet (Yaw)* : Causé par la **résistance de l'air**. On accélère les moteurs qui tournent dans le sens *inverse* de la direction voulue (pour utiliser leur couple de réaction). Les moteurs de même sens sont toujours en diagonale.



## 2. Géométrie Spatiale : Repères et Matrices

La règle d'or pour passer d'un référentiel à l'autre est de comprendre la différence entre une direction (Vecteur) et une coordonnée (Point).

* **Vecteurs (Forces, Vitesses) $\rightarrow$ Matrice 3x3 (Rotation) :** L'origine (translation) n'a pas d'importance (ex: IMU, Boussole).
* **Points (Coordonnées) $\rightarrow$ Matrice 4x4 (Homogène) :** Fusionne la rotation et la translation en une seule opération. Indispensable pour la géométrie spatiale.
* **L'astuce de la Matrice 3x3 :** Le "1" sur l'axe de rotation, les $\cos$ sur la diagonale principale, et les $\sin$ restants. **L'axe Y est le rebelle** : à cause du cycle naturel des axes, son $-\sin$ est en bas à gauche, contrairement à X et Z (en haut à droite).
* **La trinité mathématique de la rotation :** Pour une matrice de rotation pure (3x3), tourner dans l'autre sens, transposer les lignes/colonnes, ou calculer l'inverse donne **exactement le même résultat**.

$$R(-\theta) = R^T = R^{-1}$$



## 3. D'où viennent les Coordonnées ? (Matériel vs Logiciel)

* **Le LOCAL (Instantané) = Le Matériel (Capteurs) :** Dans son repère, le robot est toujours à (0, 0, 0). Les coordonnées locales sont les positions instantanées des obstacles ou des forces lues en direct par l'électronique.
* *Le Lidar / Caméra :* Chronomètre le temps de vol d'un laser et dit : "Je vois un point à 3m devant et 1m à gauche".
* *L'IMU :* Subit une force physique et dit : "Je ressens $9.81 \, m/s^2$ vers le bas de mon boîtier".


* **Le GLOBAL (Intégré) = Le Logiciel (Odométrie / EKF) :** À part le GPS, aucun capteur ne donne de position absolue. Les coordonnées globales ($x_W, y_W, \theta_W$) (le "Point Bleu" du robot) sont une **estimation mathématique** calculée en permanence en cumulant les petits déplacements depuis un point d'origine, puis corrigée.
* **Le POINT EXTERNE ($P_{global}$) :** C'est la position absolue (le "Pin Rouge" sur la carte) d'un obstacle ou d'un objectif. Elle ne vient pas du robot, mais d'un algorithme externe : une carte pré-chargée, un module SLAM/VIO, ou un clic sur QGroundControl.

## 4. La Boucle Temporelle du Filtre de Kalman (EKF)

Le Kalman est la "mémoire" du robot. Son état ($X$) vit TOUJOURS dans le Monde global. C'est indispensable pour que l'intégration physique (accélération $\rightarrow$ vitesse $\rightarrow$ position) soit juste dans le temps par rapport à un référentiel fixe.

### ÉTAPE 1 : La Prédiction ($X_{pred}$)

Le robot avance. On utilise les équations de mouvement (la cinématique) pour calculer le nouvel état théorique global.


$$X_{pred} = f(X_{prec}, U, dt)$$

### ÉTAPE 2 : L'Injection et le calcul de $h(X)$

> 🚨 **LA RÈGLE D'OR DE $h(X)$ :** Pour deviner ce que le capteur va mesurer en local, on prend **UNIQUEMENT** les composantes de notre vecteur d'état global concernées par cette mesure. Puis, on leur applique la transformation inverse appropriée :
> * Soit une **rotation transposée ($R^T$)** si la mesure est un Vecteur.
> * Soit une **matrice homogène inverse** si la mesure est un Point.
> 
> 

La Jacobienne $H$ n'est pas une matrice de chiffres fixes, c'est une grille de formules algébriques. On prend les chiffres tout chauds de $X_{pred}$ (les $x_W, y_W, \theta_W$) et on les **injecte** dans la fonction $h(X)$ pour deviner ce que le capteur devrait mesurer en local.

* **Cas 1 : Mesure d'un VECTEUR (L'Accéléromètre / IMU)**
La position spatiale n'a aucune influence. L'équation matricielle est la rotation pure :

$$h_{IMU}(X) = R^T \cdot \begin{bmatrix} \ddot{x}_W \\ \ddot{y}_W \end{bmatrix}$$


$$h_{IMU}(X) = \begin{bmatrix} \cos(\theta_W) & \sin(\theta_W) \\ -\sin(\theta_W) & \cos(\theta_W) \end{bmatrix} \cdot \begin{bmatrix} \ddot{x}_W \\ \ddot{y}_W \end{bmatrix} = \begin{bmatrix} \ddot{x}_W \cos(\theta_W) + \ddot{y}_W \sin(\theta_W) \\ -\ddot{x}_W \sin(\theta_W) + \ddot{y}_W \cos(\theta_W) \end{bmatrix}$$


* **Cas 2 : Mesure d'un POINT (La Caméra / Le Lidar)**
La distance est cruciale. On soustrait d'abord la position estimée du robot ($T_{robot\_global}$) à la position du pilier ($P_{global}$ fournie par le SLAM), PUIS on tourne :

$$h_{SLAM}(X) = R^T \cdot (P_{global} - T_{robot\_global})$$


$$h_{SLAM}(X) = \begin{bmatrix} \cos(\theta_W) & \sin(\theta_W) \\ -\sin(\theta_W) & \cos(\theta_W) \end{bmatrix} \cdot \begin{bmatrix} X_{pilier} - x_W \\ Y_{pilier} - y_W \end{bmatrix}$$



### ÉTAPE 3 : L'Update (La Correction)

On lit la vraie mesure du capteur ($z_{mesure\_brute}$), on la compare avec $h(X)$, et on calcule l'erreur ($y$) pour corriger l'état :


$$y = z_{mesure\_brute} - h(X)$$

> **💡 Kalman Linéaire vs Étendu (EKF) :**
> * **Kalman Linéaire :** Géométrie fixe ou capteur absolu (GPS). La matrice $H$ ne contient que des constantes (0, 1...). Pas besoin de recalculer $H$ à chaque boucle.
> * **EKF :** Dès que le robot tourne librement, l'angle $\theta$ s'insère dans des $\cos$ et $\sin$ (non-linéarité). On est obligé de dériver ces équations (pour obtenir la Jacobienne $H$) et de recalculer cette matrice numériquement à chaque milliseconde.
> 
> 

## 5. 🚨 DÉMARCHE CAPTEUR : Quand projeter ou non une mesure ?

Face à une nouvelle donnée capteur, posez-vous la question : **"Quel est le but de cette donnée ?"**

### Scénario A : Le but est le Filtre de Kalman (Fusion de données)

**$\rightarrow$ RÈGLE : NE JAMAIS PROJETER LA MESURE ($z$).** L'innovation se fait toujours dans le repère natif du capteur ($y = z - h(X)$). Tourner mathématiquement une mesure brute, c'est aussi devoir tourner sa matrice de covariance (son bruit), ce qui crée des instabilités numériques majeures.

1. **Garder la mesure native :** Laissez $z$ dans son repère local.
2. **Créer le traducteur $h(X)$ :** (Voir Étape 2 ci-dessus).
3. ⚠️ **LE PIÈGE 4x4 :** On ne peut PAS juste mettre un angle négatif dans une matrice 4x4 normale pour l'inverser, car cela n'inverse pas la translation. Il faut utiliser la vraie définition de l'inverse homogène :

$$H^{-1} = \begin{bmatrix} R^T & -R^T \cdot T_{robot\_global} \\ 0 & 1 \end{bmatrix}$$



### Scénario B : Le but est la Cartographie ou le Contrôle réactif (Hors Kalman)

**$\rightarrow$ RÈGLE : PROJETER LA MESURE ($z$) VERS LE MONDE.**

1. **Appliquer la rotation directe :** Multipliez la mesure par la matrice avec l'angle actuel du robot (**$+\theta$**).

* *Exemple Lidar :* $Point_{monde} = Matrice_{4x4} \cdot Point_{local\_Lidar}$
* *Exemple IMU :* $Acc_{monde} = (Matrice_{3x3} \cdot Acc_{local\_IMU}) - Gravit\acute{e}$

2. **Utiliser ROS (tf2) :** Laissez l'arbre de transformation faire ces multiplications complexes en tâche de fond pour éviter le Gimbal Lock.

## 6. La 2D n'est qu'une fraction de la 3D

* Un robot terrestre (2D) ne peut mathématiquement pas gérer le Pitch (tangage) ou le Roll (roulis), car il n'a pas d'axe Z.
* Le **Yaw (Lacet) en 3D est exactement le $\theta$ en 2D**.
* L'unique matrice de rotation 2D s'obtient simplement en gommant la 3ème ligne et la 3ème colonne de la matrice de lacet 3D ($R_z$).

## 7. 💡 Épilogue : Le "Câblage" final de la boucle EKF

*Le lien mécanique entre le vecteur d'état, l'étape de Prédiction et la matrice d'Update.*

* **Le pré-requis spatial :** Si le robot utilise un capteur spatial (Lidar, Caméra, Radar), **il est obligatoire de traquer la position globale $(x, y, z)$ dans le vecteur d'état $X$**. Sans ces variables, impossible de remplir la case "Translation" ($T_{robot\_global}$) des matrices homogènes. *(À l'inverse, une centrale inertielle AHRS peut se contenter des angles Roll/Pitch/Yaw dans son état, mais sera physiquement aveugle aux objets spatiaux).*
* **L'injection en temps réel (Le cœur de la matrice) :** À l'instant précis où le Lidar détecte un obstacle, l'algorithme :
1. Exécute l'étape **Predict** pour recracher une estimation toute fraîche du vecteur d'état ($X_{pred}$).
2. Pioche la position $(x_W, y_W)$ de ce $X_{pred}$ et l'injecte **exactement** dans le terme $T_{robot\_global}$.
3. Pioche le cap $\theta_W$ de ce $X_{pred}$ et l'injecte dans les cosinus/sinus de $R^T$.
4. La matrice $h(X)$ (et sa Jacobienne $H$) passe d'une équation théorique à une grille numérique concrète et calibrée pour *cette milliseconde exacte*. Le capteur peut alors être comparé à la réalité !



## 8. Anti-sèches Mentales (Cheat Sheet)

* **Pour un problème PID :** Visualisez une balançoire (Roll/Pitch) et pensez à la "mélasse" de l'air sur les hélices (Yaw en diagonale).
* **Vecteur vs Point :** Vecteur = $3\times3$ (Rotation pure). Point = $4\times4$ (Rotation + Translation).
* **Le Piège du mot "Translation" :** Dans l'équation de la matrice inverse, le vecteur translation $T_{robot\_global}$ contient bien les variables **globales** de l'état ($x_W, y_W$). Il représente la position absolue du robot sur la carte.
* **Sens de Rotation :**
* Local vers Global (Cartographie) = Angle direct $+\theta$.
* Global vers Local (Modèle Kalman $H$) = Angle inverse $-\theta$ (Transposée $R^T$).


* **L'astuce de l'inversion :** Dans le code, pour inverser une rotation 3x3, utilisez `.transpose()` (instantané) plutôt que `.inverse()` (plus lourd).
* **Calculer une ligne de la Jacobienne $H$ :** Les variables de l'état qui ne sont pas impliquées dans l'équation $h(X)$ (par exemple la position $x_W$ pour la mesure d'un IMU) ont une dérivée partielle de $0$.