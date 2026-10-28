# Requirements Document

## Introduction

Système de contrôle à distance d'une voiture robotique à deux moteurs DC sur un réseau Wi-Fi local, dont l'objectif scientifique est la collecte de données réelles pour un algorithme d'optimisation énergétique (RL / programmation dynamique). Le trajet est découpé en fenêtres spatiales de longueur fixe dx ; l'ESP32 émet une observation à chaque fois qu'une fenêtre se termine, et l'hôte envoie des commandes indexées par numéro de fenêtre dans un flux UDP indépendant. L'application hôte est packagée Docker et distribuée via GitHub pour être reproductible par tout utilisateur (macOS, Linux, Windows).

## Glossary

- **ESP32** : Microcontrôleur NodeMCU-32S embarqué sur la voiture, exécutant le firmware C++ PlatformIO, responsable de la lecture de l'AS5600 et de l'INA226, du calcul des intégrales locales de fenêtre et du pilotage du TB6612FNG.
- **Application_Hôte** : Application Python tournant sur l'ordinateur hôte, composée du Processus_UDP, du Processus_Contrôle, du Journal_Episodes et du Dashboard, orchestrés via Docker Compose.
- **Processus_UDP** : Processus Python dédié à la communication réseau — réception des Paquets_Observation de l'ESP32, publication dans le Flux_Observations Redis, lecture du Flux_Commandes Redis et envoi des Paquets_Commande à l'ESP32 ; émet aussi les paquets de vie hôte.
- **Processus_Contrôle** : Processus Python sans cadence propre, réveillé par chaque nouvelle entrée dans le Flux_Observations ; calcule et publie les commandes dans le Flux_Commandes via une instance de Controller.
- **Controller** : Classe abstraite Python exposant une méthode événementielle `on_observation(obs) -> list[(N, duty_pm)]`. Implémentations fournies : Manuel, Échelon/Rampe, Politique (chargée depuis fichier).
- **Bus_Partagé** : Instance Redis partagée entre tous les services. Contient deux flux : `observations` (Redis Stream) et `commandes` (Redis Stream) ; plus deux clés d'état : `derniere_observation` et `derniere_commande` (pour le Dashboard).
- **Flux_Observations** : Redis Stream nommé `observations` dans lequel le Processus_UDP publie chaque enregistrement de fenêtre reçu et validé.
- **Flux_Commandes** : Redis Stream nommé `commandes` dans lequel le Processus_Contrôle publie les commandes indexées par numéro de fenêtre.
- **Fenêtre** : Segment spatial de longueur dx mm parcouru par la voiture dans le sens avant. La fenêtre N correspond à la plage [N * dx, (N+1) * dx[. La durée d'une fenêtre est mesurée en µs par l'ESP32.
- **dx** : Longueur d'une fenêtre en mm (uint16), configurée par la commande de démarrage d'épisode. Plage valide : 50 à 1000 mm, défaut 250 mm.
- **Enregistrement_Fenêtre** : Données produites par l'ESP32 à la fin d'une fenêtre : N (uint16), duree_us (uint32), courant_moy_mA (uint16), tension_moy_mV (uint16), energie_fenetre_uJ (uint32).
- **Paquet_Observation** : Trame binaire little-endian packed envoyée par l'ESP32 vers l'hôte. Contient jusqu'à 3 Enregistrements_Fenêtre (le plus récent en premier). Champs d'enveloppe : version (uint8), type (uint8 : 0=fenêtre, 1=vie), episode_id (uint16), seq (uint16), t_esp_us (uint32), nb_enregistrements (uint8), derniere_commande_acceptee_N (uint16), flags (uint16), crc16 (uint16).
- **Paquet_Vie_ESP32** : Paquet_Observation de type 1 émis toutes les 250 ms quand aucune fenêtre ne se termine (voiture arrêtée ou bloquée). Schéma binaire distinct du type 0 : version (uint8), type=1 (uint8), episode_id (uint16), seq (uint16), t_esp_us (uint32), nb_enregistrements=0 (uint8), fenetre_courante_N (uint16), duree_ecoulee_us (uint32), flags (uint16), crc16 (uint16). Les champs derniere_commande_acceptee_N et les Enregistrements_Fenêtre sont absents ; fenetre_courante_N et duree_ecoulee_us les remplacent. Voir Req 2 §12.
- **Paquet_Commande** : Trame binaire little-endian packed envoyée par l'hôte vers l'ESP32. Types : 0=commande (jusqu'à 3 couples N/duty_pm), 1=démarrage d'épisode, 2=fin d'épisode, 3=arrêt d'urgence, 4=vie hôte. Champs communs : version (uint8), type (uint8), episode_id (uint16), seq (uint16), crc16 (uint16).
- **duty_pm** : Rapport cyclique en pour-mille (uint16, plage 0–1000), utilisé dans tous les paquets à la place d'un flottant, pour garantir l'exactitude du round-trip encode/décode.
- **Table_Commandes** : Table indexée par numéro de fenêtre stockée sur l'ESP32 (taille bornée, défaut 16 entrées) contenant les commandes futures reçues de l'hôte. Une entrée pour N est retirée de la table à la fin de la fenêtre N (lors du franchissement de la frontière N → N+1). Pendant toute la durée de la fenêtre N, l'entrée reste présente et `derniere_commande_utilisee` contient le duty_pm appliqué. Les entrées dont N est strictement inférieur à la fenêtre courante sont périmées et doivent être rejetées à la réception.
- **Règle_Arrêt_Fenêtres** : À l'entrée de chaque fenêtre N, l'ESP32 vérifie si une entrée pour N est présente dans la Table_Commandes. Si une commande pour N est présente, son duty_pm est appliqué. Si aucune commande pour N n'est présente, les moteurs sont coupés immédiatement (STBY LOW, PWM 0) et le bit "watchdog fenêtres" est positionné. La présence d'une commande pour une fenêtre N+k (k>0) ne permet pas de franchir la fenêtre N sans commande. Exemple : Table_Commandes = {9: 500, 13: 300} → entrée fenêtre 9 : commande présente, duty=500 appliqué ; entrée fenêtre 10 : aucune commande → arrêt immédiat (la commande pour 13 est ignorée pour cette décision). Cas particulier : si la Table_Commandes est vide à l'entrée de la fenêtre 0 (début d'épisode), l'ESP32 applique duty_pm=0 et positionne le bit watchdog_fenêtres, puis attend une commande valide avant toute progression de fenêtre. La reprise après un arrêt par Règle_Arrêt_Fenêtres (bit2=1) est possible uniquement par réception d'un Paquet_Commande de type 1 (démarrage d'épisode) ; une commande de type 0 seule ne réarme pas le système après cet arrêt.
- **Watchdog_Temporel** : Mécanisme de sécurité indépendant de la Règle_Arrêt_Fenêtres : si aucun Paquet_Commande **de type 0, 1, 2 ou 3** (c'est-à-dire hors type 4 vie hôte) avec un CRC valide et un episode_id correspondant n'est reçu pendant watchdog_temporel_s secondes (configurable dans config.h, défaut 3 s), les moteurs sont coupés et le bit "watchdog temporel" est positionné dans flags. Reprise automatique à la réception d'un Paquet_Commande de type 0 valide pour la fenêtre courante ou supérieure, ou d'un Paquet_Commande de type 1 (démarrage d'épisode). Un Paquet_Commande de type 4 (vie hôte) ne remet pas à zéro le compteur du Watchdog_Temporel.
- **Marge_Avance** : Pour chaque commande envoyée, écart entre le numéro de fenêtre cible N_cmd et la fenêtre courante N_esp au moment de l'envoi UDP, affiché en direct dans le Dashboard.
- **Horizon_Max** : Nombre maximal de fenêtres d'avance accepté par l'ESP32 (configurable dans config.h, défaut 15). Une commande dont N > fenêtre courante + Horizon_Max est rejetée.
- **Paramètre_M** : Nombre de fenêtres d'avance cible du Controller (configurable, défaut 3). Documente le compromis entre latence de décision et risque de commandes périmées.
- **Episode** : Séquence de fenêtres démarrée par une commande de type 1 et arrêtée par une commande de type 2 ou 3, identifiée par un episode_id unique. Distance maximale : distance_max_mm (uint32). L'episode_id est généré et persisté exclusivement par l'Application_Hôte (stocké dans Redis ou dans un fichier de compteur) afin d'éviter les collisions en cas de redémarrage. L'ESP32 ne génère pas d'episode_id.
- **Journal_Episodes** : Consommateur indépendant (groupe de consommateurs Redis séparé) du Flux_Observations, écrivant un CSV par épisode.
- **Dashboard** : Interface web Plotly Dash exposée sur un port configurable, affichant les données en direct et les épisodes enregistrés.
- **Simulateur** : Service Docker optionnel (profil "simulateur") émulant l'ESP32 avec le générateur pseudo-aléatoire commun, la même loi vitesse = k * duty, la même Table_Commandes et les mêmes règles d'arrêt que le firmware. Pas de modèle moteur ni de batterie.
- **Mode_Test_Capteurs** : Mode firmware activé par le flag de compilation `-DMODE_TEST_CAPTEURS`. L'AS5600 et l'INA226 ne sont pas lus ; courant et tension sont générés par le générateur pseudo-aléatoire commun ; la vitesse simulée vaut k * duty_pm / 1000 (m/s, k configurable) et la durée de fenêtre vaut `(dx_mm * 1_000_000) / (k * duty_pm / 1000)` µs (si duty_pm=0, aucune fenêtre ne se termine).
- **Générateur_PRN** : Générateur pseudo-aléatoire de type LCG (multiplicateur 1664525, incrément 1013904223, modulo 2^32) implémenté à l'identique en C++ et en Python pour garantir des séquences identiques à graine égale.
- **AS5600** : Encodeur magnétique angulaire I2C (adresse 0x36) mesurant la position de la roue. Lu toutes les 5 ms sur l'ESP32 avec gestion du franchissement de zéro et hystérésis de frontière de fenêtre.
- **INA226** : Capteur de puissance I2C (adresse 0x40, shunt 0,1 Ω) mesurant tension et courant. Lu à ~100 Hz sur l'ESP32. La plage uint16 (0–65535 mA) est compatible avec les courants typiques de la voiture robotique. Si le nombre d'échantillons invalides k dans une fenêtre est strictement inférieur à 30 % du nombre total d'échantillons n (k < 0.3 × n), la fenêtre reste valide et les valeurs conservées (hold) sont utilisées pour les échantillons manquants. Si k ≥ 0.3 × n, le bit1 est positionné à 0 (INA226 invalide) pour l'ensemble de la fenêtre et valide=0 dans le CSV.
- **TB6612FNG** : Driver moteur pilotant deux moteurs DC en rapport cyclique PWM.
- **flags_obs** : Champ uint16 du Paquet_Observation. Bits : bit0=AS5600_valide, bit1=INA226_valide, bit2=watchdog_fenêtres_actif, bit3=watchdog_temporel_actif, bit4=surintensité_détectée, bit5=aimant_AS5600_hors_plage, bit6=recul_détecté, bit7=fin_de_distance_atteinte.
- **Docker_Compose** : Orchestrateur multi-conteneurs (services : redis, udp, controle, dashboard, simulateur en profil).
- **PlatformIO** : Environnement de développement firmware. Configuration dans `platformio.ini`.
- **config.h** : Fichier de configuration du firmware ESP32 : SSID, mot de passe Wi-Fi, IP de l'hôte, port UDP hôte, port UDP ESP32, dx par défaut, Horizon_Max (défaut 15), seuil watchdog temporel, broches, duty_max, seuil de courant (défaut 500 mA), diamètre de roue, adresses I2C, paramètre k du Mode_Test_Capteurs, graine du Générateur_PRN.
- **valide** : Colonne entière du CSV indiquant la qualité d'un enregistrement de fenêtre. Valeurs : 1 = reçu directement, 2 = récupéré via redondance glissante, 0 = jamais reçu (fenêtre manquante).

- **derniere_commande_utilisee** : État interne de l'ESP32 contenant le duty_pm effectivement appliqué lors de la transition vers la dernière fenêtre ayant reçu une commande valide. Mis à jour uniquement lorsqu'une commande est effectivement appliquée (pas à la réception d'un paquet). Distinct de la Table_Commandes et de derniere_commande_acceptee_N.
- **Commande_Valide** : Entrée (N, duty_pm) d'un Paquet_Commande de type 0 acceptée dans la Table_Commandes : N ≥ fenêtre courante, N ≤ fenêtre courante + Horizon_Max, CRC valide, episode_id correspondant.
- **Paquet_Vie_Hôte** : Paquet_Commande de type 4. Indique uniquement que l'hôte est actif. Ne constitue pas une Commande_Valide. Ne met pas à jour derniere_commande_utilisee. N'empêche pas l'arrêt par la Règle_Arrêt_Fenêtres. Ne remet pas à zéro le compteur du Watchdog_Temporel.
- **Clé_Fenêtre** : Couple `(episode_id, N)` identifiant de façon unique une fenêtre dans l'ensemble des épisodes. Deux enregistrements ayant la même Clé_Fenêtre représentent la même fenêtre logique.

---

## Requirements

### Requirement 1: Flux UDP indépendants et communication par fenêtres

**User Story:** As an operator, I want the ESP32 to send a window observation at the end of each spatial window and the host to send commands on an independent UDP channel, so that both communication directions are decoupled and the system is not blocked by network jitter.

#### Acceptance Criteria

1. WHEN a spatial window of dx mm is completed by the ESP32 (detected via the AS5600), THE ESP32 SHALL emit a Paquet_Observation of type 0 (fenêtre) to the host's UDP port (configurable in config.h, default 8888).
2. WHILE no spatial window has completed and no Paquet_Observation has been sent for 250 ms, THE ESP32 SHALL emit a Paquet_Observation of type 1 (vie) carrying the current partial window state and flags.
3. THE Processus_UDP SHALL send Paquet_Commande packets to the ESP32's UDP port (configurable in config.h, default 8889) independently of receiving Paquet_Observation packets.
4. WHEN no Paquet_Observation has yet been received from the ESP32 during the current session, THE Processus_UDP SHALL NOT send any Paquet_Commande.
5. WHEN a Paquet_Observation is received, THE Processus_UDP SHALL learn the ESP32's IP address from the UDP source address of that packet and use it for all subsequent Paquet_Commande transmissions.
6. THE Processus_UDP SHALL send at least one Paquet_Commande of type 4 (vie hôte) per second even when the Processus_Contrôle produces no new command.
7. EACH Paquet_Observation SHALL include up to 3 window records (the most recent first) to provide sliding redundancy.
8. EACH Paquet_Commande of type 0 (commande) SHALL include up to 3 command entries (N, duty_pm) to provide sliding redundancy.
9. IF a packet is received with a mismatched episode_id, unknown version, incorrect size, or invalid CRC16, THEN THE receiver SHALL reject the packet, increment a rejected-packet counter, and not apply any command or record.
10. THE Processus_UDP SHALL deduplicate window records on the receiving side using the Clé_Fenêtre (episode_id, N), keeping the first occurrence and discarding subsequent duplicates for the same Clé_Fenêtre.
11. THE ESP32 SHALL deduplicate command entries on the receiving side by window number N, replacing an existing entry with a newer one for the same N.
12. WHEN a Paquet_Observation of type 1 (vie) is received by the Processus_UDP, THE Journal_Episodes SHALL NOT write any CSV row for it; the Paquet_Vie_ESP32 is used only to update the derniere_observation state key and the Dashboard health indicators.

---

### Requirement 2: Format binaire des paquets (entiers, little-endian packed, schéma unique)

**User Story:** As a developer, I want all packet fields to use integer types in a single shared schema, so that encode/decode round-trips are exact and there is no floating-point rounding in distances or energy.

#### Acceptance Criteria

1. THE Enregistrement_Fenêtre SHALL contain exactly the following fields in order: N (uint16), duree_us (uint32), courant_moy_mA (uint16), tension_moy_mV (uint16), energie_fenetre_uJ (uint32).
2. THE Paquet_Observation envelope SHALL contain exactly the following fields in order: version (uint8), type (uint8), episode_id (uint16), seq (uint16), t_esp_us (uint32), nb_enregistrements (uint8), derniere_commande_acceptee_N (uint16), flags (uint16), then nb_enregistrements × Enregistrement_Fenêtre, then crc16 (uint16).
3. THE Paquet_Commande of type 0 (commande) SHALL contain: version (uint8), type (uint8), episode_id (uint16), seq (uint16), nb_commandes (uint8), then nb_commandes × (N: uint16, duty_pm: uint16), then crc16 (uint16).
4. THE Paquet_Commande of type 1 (démarrage d'épisode) SHALL contain: version, type, episode_id, seq, dx_mm (uint16), distance_max_mm (uint32), crc16.
5. THE Paquet_Commande of type 2 (fin d'épisode) SHALL contain: version, type, episode_id, seq, crc16.
6. THE Paquet_Commande of type 3 (arrêt d'urgence) SHALL contain: version, type, episode_id, seq, crc16.
7. THE Paquet_Commande of type 4 (vie hôte) SHALL contain: version, type, episode_id, seq, crc16.
8. THE packet schema SHALL be defined exactly once in a shared source (C++ header for the firmware and a Python module generated or verified by a coherence test).
9. THE duty_pm field SHALL represent the duty cycle in per-mille (uint16, range 0–1000) in all packets, replacing any floating-point duty representation.
10. FOR ALL valid Enregistrements_Fenêtre encoded by the ESP32, deserialization by the Processus_UDP followed by re-serialization SHALL produce a binary-identical byte sequence (round-trip property).
11. THE coherence test SHALL verify that the C++ struct and the Python module define identical field sizes, types, and order.
12. THE Paquet_Observation of type 1 (vie) SHALL contain: version (uint8), type=1 (uint8), episode_id (uint16), seq (uint16), t_esp_us (uint32), nb_enregistrements=0 (uint8), fenetre_courante_N (uint16), duree_ecoulee_us (uint32), flags (uint16), crc16 (uint16). The fields derniere_commande_acceptee_N and nb_enregistrements×Enregistrement_Fenêtre are absent; fenetre_courante_N and duree_ecoulee_us replace them.
13. THE CRC16 field SHALL be computed using the CRC-16/CCITT-FALSE algorithm (polynomial 0x1021, initial value 0xFFFF, no bit reversal) over all bytes of the packet preceding the CRC16 field. The receiver SHALL reject any packet whose computed CRC16 does not match the received CRC16 field.

---

### Requirement 3: Détection de fin de fenêtre et mesures de fenêtre sur l'ESP32

**User Story:** As a researcher, I want the ESP32 to detect window boundaries using the AS5600 and compute per-window averages and integrals locally, so that each window record contains accurate, gap-free energy and duration data.

#### Acceptance Criteria

1. WHILE the ESP32 is active in an episode, THE ESP32 SHALL read the AS5600 (I2C address 0x36) every 5 ms to accumulate forward distance, applying zero-crossing handling and a hysteresis threshold at window boundaries to avoid spurious transitions.
2. WHEN the accumulated forward distance reaches dx mm, THE ESP32 SHALL record the window end: store N, measure duree_us as the elapsed time since the previous window boundary, compute courant_moy_mA as the mean of all INA226 current samples within the window, compute tension_moy_mV as the mean of all INA226 voltage samples within the window, and compute energie_fenetre_uJ as the integral of power over the window duration.
3. THE ESP32 SHALL compute vitesse_moy as dx / duree_us on the host side only; THE ESP32 SHALL NOT transmit a speed field.
4. THE ESP32 SHALL NOT maintain cumulative energy or cumulative position across windows; all cumulative values SHALL be computed by the Application_Hôte from the per-window records.
5. WHILE the ESP32 is active, THE ESP32 SHALL read the INA226 (I2C address 0x40, shunt 0.1 Ω) at approximately 100 Hz to sample current and voltage for per-window averaging.
6. IF an I2C read error is detected on the AS5600, THEN THE ESP32 SHALL set bit0=0 (AS5600 invalide) in flags_obs and hold the last valid position for distance accumulation.
7. IF an I2C read error is detected on the INA226, THEN THE ESP32 SHALL set bit1=0 (INA226 invalide) in flags_obs and hold the last valid current and voltage values for averaging. IF the number of invalid INA226 samples k within the window reaches or exceeds 30 % of the total number of samples n (k ≥ 0.3 × n), THEN THE ESP32 SHALL set bit1=0 (INA226 invalide) for the entire window.
8. IF the AS5600 detects a magnet out of range, THEN THE ESP32 SHALL set bit5=1 (aimant_hors_plage) in flags_obs.
9. IF the ESP32 detects backward motion (negative AS5600 delta), THEN THE ESP32 SHALL set bit6=1 (recul_détecté) in flags_obs and ignore the backward distance for forward accumulation.
10. WHEN the cumulative forward distance since the start of an episode reaches distance_max_mm, THE ESP32 SHALL stop the motors, set bit7=1 (fin_de_distance) in flags_obs, and include this state in the next Paquet_Observation.
11. WHEN computing per-window averages, THE ESP32 SHALL count the number of INA226 read errors k and the total number of expected samples n within the window. IF k < 0.3 × n, THE ESP32 SHALL use the last valid held value for missing samples and keep bit1=1 (INA226 valide). IF k ≥ 0.3 × n, THE ESP32 SHALL set bit1=0 (INA226 invalide) for the window, and the Journal_Episodes SHALL set valide=0 for that row.

---

### Requirement 4: Pilotage des moteurs et comportement de l'ESP32 face aux commandes

**User Story:** As an operator, I want the ESP32 to apply spatially-indexed commands to its motors with clear safety rules, so that the vehicle follows the commanded duty cycle and stops safely when commands are missing or the connection is lost.

#### Acceptance Criteria

1. THE ESP32 SHALL maintain a Table_Commandes indexed by window number N with a bounded size (default 16 entries).
2. WHEN a Paquet_Commande of type 0 is received, THE ESP32 SHALL store each (N, duty_pm) entry in the Table_Commandes if N is greater than or equal to the current window number (a command for N equal to the current window number is applicable if received before the window boundary is crossed) and N is less than or equal to the current window number plus Horizon_Max (default 15); otherwise THE ESP32 SHALL discard the entry.
3. WHEN a command entry for a window number already present in the Table_Commandes is received, THE ESP32 SHALL replace the existing entry with the new one.
4. WHEN the ESP32 enters window N, IF an entry for N is present in the Table_Commandes, THEN THE ESP32 SHALL apply its duty_pm and update derniere_commande_utilisee with that duty_pm, and retain the entry for N in the Table_Commandes until the end of window N (when the N→N+1 boundary is crossed).
5. WHEN the ESP32 crosses the boundary from window N to window N+1, THE ESP32 SHALL remove the entry for N from the Table_Commandes (if still present).
6. WHEN the ESP32 enters window N, IF no entry for N is present in the Table_Commandes (Règle_Arrêt_Fenêtres), THEN THE ESP32 SHALL stop the motors immediately (STBY LOW, PWM 0) and set bit2=1 (watchdog_fenêtres) in flags_obs. The presence of an entry for any window N+k (k>0) SHALL NOT prevent this stop.
7. IF the Table_Commandes is empty when the episode starts (window 0), THE ESP32 SHALL apply duty_pm=0 and set bit2=1 (watchdog_fenêtres) in flags_obs, and SHALL wait for a valid command before allowing window progression.
8. IF no Paquet_Commande of type 0, 1, 2, or 3 with a valid CRC and matching episode_id has been received for watchdog_temporel_s seconds (configurable in config.h, default 3 s), THEN THE ESP32 SHALL stop the motors (STBY LOW, PWM 0) and set bit3=1 (watchdog_temporel) in flags_obs. A Paquet_Commande of type 4 (vie hôte) SHALL NOT reset the Watchdog_Temporel timer.
9. WHEN the ESP32 is stopped due to the Règle_Arrêt_Fenêtres (bit2=1 watchdog_fenêtres), THE ESP32 SHALL resume motor control only upon receiving a Paquet_Commande of type 1 (démarrage d'épisode); a Paquet_Commande of type 0 alone SHALL NOT resume motor control after a Règle_Arrêt_Fenêtres stop.
10. WHEN a valid Paquet_Commande of type 0, 1, 2, or 3 for an active episode is received after a watchdog_temporel condition, THE ESP32 SHALL resume motor control and clear the watchdog_temporel bit in flags_obs. A Paquet_Commande of type 4 (vie hôte) SHALL NOT trigger a watchdog resume.
11. WHEN a Paquet_Commande of type 3 (arrêt d'urgence) is received, THE ESP32 SHALL stop the motors immediately (STBY LOW, PWM 0) regardless of any planned command.
12. THE ESP32 SHALL clamp any duty_pm to the range [0, duty_max] defined in config.h before applying it to the TB6612FNG.
13. IF the current measured by the INA226 exceeds the configurable overcurrent threshold defined in config.h (default 500 mA), THEN THE ESP32 SHALL immediately stop the motors (STBY LOW, PWM 0) and set bit4=1 (surintensité) in flags_obs.
14. AT ESP32 startup, THE ESP32 SHALL initialize the motors to stopped state (STBY LOW, PWM 0) before any network connection is established.
15. THE ESP32 SHALL disable Wi-Fi sleep mode (WiFi.setSleep(false)) to suppress latency spikes.
16. WHEN a Paquet_Commande of type 1 (démarrage d'épisode) is received, THE ESP32 SHALL reset the window counter to 0, clear the Table_Commandes, record dx_mm and distance_max_mm, and set the episode_id for subsequent packets.
17. WHEN a Paquet_Commande of type 2 (fin d'épisode) is received, THE ESP32 SHALL stop the motors and cease emitting Paquets_Observation until a new episode start command is received.
18. A Paquet_Commande of type 4 (vie hôte) SHALL NOT reset the Watchdog_Temporel timer, SHALL NOT be considered a Commande_Valide, and SHALL NOT prevent motor stop when no command is available for the current window.

---

### Requirement 5: Bus partagé Redis avec flux et clés d'état

**User Story:** As a developer, I want the shared Redis bus to expose both event streams and current-state keys, so that the control process reacts to every window event without polling while the dashboard can read the latest state at any time.

#### Acceptance Criteria

1. THE Bus_Partagé SHALL use Redis as the sole inter-process communication mechanism, using two Redis Streams (Flux_Observations and Flux_Commandes) and two state keys (derniere_observation and derniere_commande).
2. THE Processus_UDP SHALL publish each validated window record to the Flux_Observations Redis Stream after receiving and validating a Paquet_Observation.
3. THE Processus_UDP SHALL also update the derniere_observation state key with the deserialized Paquet_Observation content and a host timestamp after each validated Paquet_Observation.
4. THE Processus_Contrôle SHALL read the Flux_Observations Redis Stream using a blocking read (no polling) and be woken up by each new entry.
5. THE Processus_Contrôle SHALL publish command entries (N, duty_pm) to the Flux_Commandes Redis Stream and update the derniere_commande state key after each command decision.
6. THE Processus_UDP SHALL consume the Flux_Commandes Redis Stream and send the corresponding Paquet_Commande to the ESP32.
7. THE Journal_Episodes SHALL consume the Flux_Observations Redis Stream using a separate Redis consumer group, independently of the Processus_Contrôle consumer group.
8. IF Redis is not accessible at service startup, THEN THE service SHALL abort startup and log a descriptive error message.
9. IF the Processus_Contrôle crashes or restarts, THEN all window observations already published to the Flux_Observations SHALL remain in the stream and no command SHALL be produced for the missed observations.
10. THE Processus_Contrôle SHALL be able to pre-load commands for windows 0 to M (configurable Paramètre_M, default 3) at the start of an episode before any Paquet_Observation is received.
11. THE Bus_Partagé SHALL support at least 100 read/write operations per second without measurable degradation.
12. THE Application_Hôte SHALL generate episode_id values using a persistent counter stored in Redis (or a local file), incrementing it at each new episode start, so that episode_id values remain unique across restarts.

---

### Requirement 6: Interface Controller événementielle et algorithmes de contrôle

**User Story:** As a developer, I want a replaceable Controller interface driven by window events rather than a fixed clock, so that I can implement and test energy optimization algorithms that react to spatial progress.

#### Acceptance Criteria

1. THE Processus_Contrôle SHALL use a Controller instance exposing the method `on_observation(obs: dict) -> list[tuple[int, int]]` returning a list of (N, duty_pm) pairs.
2. THE obs parameter SHALL contain at minimum: N (window number), duree_us, courant_moy_mA, tension_moy_mV, energie_fenetre_uJ, and the controller's current advance margin M.
3. THE Application_Hôte SHALL provide three Controller implementations: Manuel (returns a constant configurable duty_pm), Échelon/Rampe (programmed command profile for data collection), and Politique (loads an algorithm from an external file).
4. THE Controller implementation in use SHALL be selectable by configuration without modifying the Processus_Contrôle code or the Redis access layer.
5. THE Processus_Contrôle SHALL allow the Controller to publish commands at any time, not only in response to an observation event.
6. IF the Processus_Contrôle raises an unhandled exception, THEN THE Processus_Contrôle SHALL log the error with a full stack trace and restart automatically within 1 second.

---

### Requirement 7: Journal d'épisodes (CSV par fenêtre)

**User Story:** As a researcher, I want each episode recorded in a complete CSV with one row per window, so that I have structured data for training energy optimization algorithms.

#### Acceptance Criteria

1. THE Journal_Episodes SHALL write one CSV file per episode in a Docker-mounted volume directory, with the columns: episode_id, N, x_mm, duree_us, vitesse_moy_m_s, courant_moy_mA, tension_moy_mV, puissance_moy_mW, energie_fenetre_uJ, energie_cumulee_uJ, temps_cumule_us, duty_commande_pm, flags, valide (INT: 1=reçu directement, 2=récupéré via redondance glissante, 0=jamais reçu).
2. THE Journal_Episodes SHALL compute: vitesse_moy_m_s as `(dx_mm * 1000) / duree_us` (result in m/s, dx_mm in mm, duree_us in µs); puissance_moy_mW as `(energie_fenetre_uJ * 1000) / duree_us` (result in mW); energie_cumulee_uJ as the running sum of energie_fenetre_uJ since episode start; and temps_cumule_us as the running sum of duree_us since episode start.
3. WHEN a window record is reconstructed via sliding redundancy (not received directly but recovered from a later packet), THE Journal_Episodes SHALL set valide=2 **provisionally**, pending possible receipt of the direct record.
4. WHEN a window is never received even after redundancy recovery, THE Journal_Episodes SHALL write a row with valide=0 and leave measured fields empty, without deleting the row.
5. WHEN a window record has any active safety flag (bit2=1 watchdog_fenêtres, bit3=1 watchdog_temporel, bit4=1 surintensité, bit0=0 AS5600 invalide, or bit1=0 INA226 invalide), THE Journal_Episodes SHALL set valide=0 for that row without deleting it.
6. WHEN an episode is started from the Dashboard, THE Journal_Episodes SHALL create a new CSV file with a unique name including the date and episode name.
7. WHEN an episode is stopped from the Dashboard, THE Journal_Episodes SHALL close the CSV file and write the episode metadata (creation date, active configuration parameters, name, label) to an associated JSON file.
8. THE Journal_Episodes SHALL only write rows for Paquet_Observation of type 0 (fenêtre); Paquets_Vie_ESP32 (type 1) SHALL be ignored by the Journal_Episodes.
9. IF a window record with the same Clé_Fenêtre (episode_id, N) is received after being previously recovered via redundancy (valide=2), THEN THE Journal_Episodes SHALL update the existing row (valide becomes 1 if the direct record has no active safety flags, or 0 if it does) and SHALL NOT create a second row or double-count energie_fenetre_uJ in energie_cumulee_uJ.

---

### Requirement 8: Dashboard web

**User Story:** As an operator, I want a web dashboard accessible in my browser with position-based charts and spatial health indicators, so that I can monitor the system in real time and manage recording episodes.

#### Acceptance Criteria

1. THE Dashboard SHALL be a Plotly Dash application accessible on a configurable port (default documented in docker-compose.yml).
2. THE Dashboard SHALL display live charts (updated on each new window event) with both time and position (x_mm) as selectable x-axes, for: vitesse_moy_m_s, courant_moy_mA, tension_moy_mV, puissance_moy_mW, energie_cumulee_uJ, and duty_commande_pm.
3. THE Dashboard SHALL display the following health indicators: missing windows count, recovered windows count, Marge_Avance (current value and trend), stale commands count (commandes périmées), window number triggering the Règle_Arrêt_Fenêtres, Watchdog_Temporel state, AS5600 magnet state (bit5), and per-flag bit status.
4. THE Dashboard SHALL expose the following controls: start/stop episode recording (sending type-1 and type-2 Paquet_Commande), emergency stop (type-3 Paquet_Commande), and episode label (à garder / à supprimer).
5. THE Dashboard SHALL list recorded episodes with the ability to replay an episode by displaying curves from the CSV file.
6. THE Dashboard SHALL remain responsive at the window event rate without blocking other services.

---

### Requirement 9: Configuration réseau et démarrage de l'ESP32

**User Story:** As an operator, I want to configure the host IP in the firmware and both UDP ports in configuration files, so that the connection is reliable without requiring a static IP on the ESP32.

#### Acceptance Criteria

1. THE ESP32 firmware SHALL read the host IP address and the host UDP target port from config.h at compile time.
2. THE ESP32 firmware SHALL listen for Paquet_Commande on a configurable UDP port defined in config.h (default 8889).
3. THE Application_Hôte SHALL read its UDP listening port from a single configuration file at startup.
4. IF the IP address or port configured in config.h is syntactically invalid, THEN THE firmware SHALL refuse to compile using a static_assert or equivalent compile-time check.
5. THE documentation SHALL specify that the host computer's IP address must be fixed via DHCP reservation on the router.

---

### Requirement 10: Déploiement Docker

**User Story:** As a user, I want to start the entire application with a single Docker command, so that I can deploy the system without manually installing Python dependencies.

#### Acceptance Criteria

1. THE docker-compose.yml SHALL define four core services: redis, udp, controle, dashboard (the simulateur is an optional profile).
2. WHEN the command `docker compose up` is executed from the repository root, THE Application_Hôte SHALL start fully without additional manual intervention.
3. THE docker-compose.yml SHALL publish the host UDP listening port explicitly (e.g., `8888:8888/udp`) to ensure compatibility with macOS and Windows where `network_mode: host` is not supported.
4. THE documentation SHALL distinguish the Linux network configuration (optional network_mode: host) from the macOS/Windows configuration (mandatory port publishing).
5. THE docker-compose.yml SHALL mount the configuration file and the episodes directory as volumes to allow modification without rebuilding images.
6. THE docker-compose.yml SHALL set `restart: unless-stopped` on all services.
7. THE docker-compose.yml SHALL declare the redis service as a dependency of the udp, controle, and dashboard services.

---

### Requirement 11: Firmware PlatformIO

**User Story:** As a contributor, I want the firmware to be compilable with PlatformIO without modification for the target hardware, so that the project can be reproduced from the GitHub repository.

#### Acceptance Criteria

1. THE repository SHALL contain a `platformio.ini` file configured for the esp32dev board, the arduino framework, and with all library dependencies pinned to exact versions.
2. THE firmware SHALL read all its configuration from `config.h`: SSID, Wi-Fi password, host IP, host UDP port, ESP32 UDP port, dx default, Horizon_Max (default 15), watchdog_temporel_s, motor pins, duty_max, current threshold (default 500 mA), wheel diameter, I2C addresses, Mode_Test_Capteurs parameter k, and Générateur_PRN seed.
3. THE firmware SHALL use the following default pin assignments (overridable in config.h): PWMA=25, AIN1=26, AIN2=27, PWMB=33, BIN1=14, BIN2=13, STBY=32, SDA=21, SCL=22.
4. THE repository SHALL contain a `config.h.example` file documenting all parameters with their default values.

---

### Requirement 12: Simulateur ESP32 et Mode Test Firmware

**User Story:** As a developer, I want a Python simulator and a firmware test mode that share the same pseudo-random generator, so that I can validate the full software chain without physical hardware and verify output signals with a multimeter.

#### Acceptance Criteria

1. THE Simulateur SHALL be activatable via `docker compose --profile simulateur up` without modifying docker-compose.yml.
2. THE Simulateur SHALL send Paquet_Observation packets in the exact format defined in Requirement 2 and apply received Paquet_Commande packets using the same Table_Commandes and Règle_Arrêt_Fenêtres as the firmware.
3. THE Simulateur SHALL use the Générateur_PRN (LCG: multiplier 1664525, increment 1013904223, modulo 2^32) to generate courant_moy_mA and tension_moy_mV values, with a configurable seed.
4. THE Simulateur SHALL compute window duration as `duree_us = (dx_mm * 1_000_000) / (k * duty_pm / 1000)` (in µs, where dx_mm is in mm, k in m/s/full-scale, duty_pm in 0–1000) using the same k parameter as the firmware Mode_Test_Capteurs, with no motor or battery model. IF duty_pm is 0, THE Simulateur SHALL NOT advance the window (no window end event is emitted) and SHALL emit Paquet_Vie_ESP32 packets until a non-zero command is received.
5. THE Simulateur SHALL support a configurable packet loss rate to test redundancy and safety rules.
6. WHERE the `-DMODE_TEST_CAPTEURS` compile flag is set, THE ESP32 firmware SHALL NOT read the AS5600 or INA226 and SHALL generate courant_moy_mA and tension_moy_mV using the Générateur_PRN with the seed defined in config.h.
7. WHERE the `-DMODE_TEST_CAPTEURS` compile flag is set, THE ESP32 firmware SHALL compute window duration as `duree_us = (dx_mm * 1_000_000) / (k * duty_pm / 1000)` using the configurable k parameter from config.h, with a value low enough that each window lasts several seconds (readable with a 1 Hz multimeter). IF duty_pm is 0, THE ESP32 SHALL NOT advance the window and SHALL emit Paquet_Vie_ESP32 packets until a non-zero command is received.
8. WHERE the `-DMODE_TEST_CAPTEURS` compile flag is set, THE ESP32 SHALL produce on PWMA/PWMB a mean voltage equal to (duty_pm / 1000) × 3.3 V, measurable with a multimeter; STBY and AIN/BIN SHALL output 0 V or 3.3 V; and motor cutoff on watchdog SHALL produce 0 V on STBY.
9. THE documentation SHALL include a step-by-step multimeter test procedure for Mode_Test_Capteurs.
10. THE coherence test SHALL verify that the Simulateur's Générateur_PRN and the firmware's Générateur_PRN produce identical sequences for the same seed (LCG: multiplier 1664525, increment 1013904223, modulo 2^32).

---

### Requirement 13: Suite de tests automatisés

**User Story:** As a developer, I want an automated test suite covering the packet codec, Redis bus, window stop rule, and end-to-end behavior, so that regressions are detected before deployment.

#### Acceptance Criteria

1. THE test suite SHALL include unit tests for the packet codec: serialization, deserialization, round-trip (FOR ALL valid window records, encode then decode SHALL return an identical record), and rejection of packets with bad version, wrong size, or invalid CRC16.
2. THE test suite SHALL include unit tests for the Bus_Partagé: write to Flux_Observations, read by two independent consumer groups, write to Flux_Commandes, and read by Processus_UDP.
3. THE test suite SHALL include a test for the Règle_Arrêt_Fenêtres using the Simulateur: given Table_Commandes = {9: 500, 13: 300}, verify that the Simulateur stops at window entry 10 (no command for window 10) and sets bit2=1 in flags_obs, and that the command for window 13 does not prevent this stop.
4. THE test suite SHALL include a test for the Watchdog_Temporel using the Simulateur: verify that the Simulateur stops after 3 s without a valid Paquet_Commande.
5. THE test suite SHALL include a test for sliding redundancy: simulate a Paquet_Observation carrying window N in position 2 (not position 1), verify that the Processus_UDP recovers and records window N.
6. THE test suite SHALL include an end-to-end test with the Simulateur covering a complete episode: start, window recording in CSV, stop, and JSON metadata file creation.
7. THE test suite SHALL include the Générateur_PRN coherence test: verify that the Python implementation and the C++ reference (via a fixture file) produce identical sequences for at least 1000 values with the same seed.
8. THE test suite SHALL include the C++/Python schema coherence test: verify that field sizes, types, and order are identical in both implementations.
9. THE test suite SHALL include a test verifying that a Paquet_Commande of type 4 (vie hôte) does not reset the Watchdog_Temporel, does not update derniere_commande_utilisee, and does not prevent motor stop when no command is available for the current window.
10. THE test suite SHALL include a test verifying that after a Règle_Arrêt_Fenêtres stop (bit2=1), a Paquet_Commande of type 0 does not resume motor control, and that only a Paquet_Commande of type 1 resumes the episode.
11. THE test suite SHALL include a test for the INA226 partial-validity rule: simulate a window with k=3 invalid samples out of n=9 total (k/n = 0.33 ≥ 0.3); verify that bit1=0 and valide=0. Simulate a window with k=2 invalid samples out of n=9 (k/n ≈ 0.22 < 0.3); verify that bit1=1 and valide=1 (assuming no other active safety flags).

---

### Requirement 14: Journalisation et observabilité

**User Story:** As a developer, I want the application to log key events and metrics to stdout, so that problems can be diagnosed through Docker logs.

#### Acceptance Criteria

1. THE Processus_UDP SHALL log to stdout every 5 seconds: windows received per second, windows recovered via redundancy, windows lost (never received), Paquets_Observation rejected, Paquets_Commande sent, and stale commands count.
2. WHEN a network error occurs, THE Processus_UDP SHALL log the error with a timestamp, error type, and source IP address.
3. THE Application_Hôte SHALL log a warning if the Marge_Avance drops below 1 window, including the current N_esp and N_cmd values.
4. THE Application_Hôte SHALL write all logs to stdout to be captured by Docker without additional configuration.

---

### Requirement 15: Documentation GitHub

**User Story:** As a user, I want to clone the repository and start the system in under 15 minutes, so that I can reproduce the project on my own robotic car.

#### Acceptance Criteria

1. THE README SHALL cover: required hardware list (ESP32 NodeMCU-32S, AS5600, INA226, TB6612FNG, DC motors, battery), power and logic wiring diagram, calibration (wheel diameter, AS5600 magnet air gap), flashing with PlatformIO, and Docker startup.
2. THE README SHALL document the first-use procedure: test with the Simulateur, then with the real car.
3. THE README SHALL document the network configuration for macOS/Windows and for Linux.
4. THE README SHALL document the multimeter test procedure using Mode_Test_Capteurs.
5. THE repository SHALL contain a Controller extension point documentation describing how to implement a new algorithm, including the on_observation interface and the Paramètre_M advance margin.
6. THE repository SHALL contain a `docker-compose.yml` at the root, a `config.h.example` in the firmware directory, and a `.env.example` listing Docker environment variables.

---

## Out of Scope

- Steering / servo motor control
- Camera integration
- IMU integration
- Current regulation loop on the ESP32
- Multi-vehicle management
- Reverse driving (backward motion is detected and ignored for distance accumulation)
- Motor or battery model in the Simulateur (only the kinematic law vitesse = k * duty_pm / 1000 is modeled)
- Any functionality not listed in Requirements 1 to 15 without an update to this document

---

