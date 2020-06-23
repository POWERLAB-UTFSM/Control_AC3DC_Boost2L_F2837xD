# Control_AC3DC_Boost2L_F2837xD

## Resumen
Este repositorio corresponde al código para el control y supervisión de un rectificador trifásico Boost de 2 niveles, usando el microcontrolador TMS320F2837xD.

## Pertenencia a proyecto
* **Código Proyecto**: FONDEF IT 18I0090
* **Título Proyecto**: 
* **Investigadores responsables**:
    * Ana Llor (Directora)
    * Freddy Flores
    * Marcelo Pérez
    * Samir Kouro
    * Alan Wilson
    * Sebastián Rivera
* **Estado**: En desarrollo
* **Inicio**: Junio 2019
* **Término**: -

## Autores del repositorio y ramas
* [Alan Wilson](https://github.com/orgs/POWERLAB-UTFSM/people/Alan-H-Wilson-V) (master, ALL)
* [Matías Licanqueo](https://github.com/orgs/POWERLAB-UTFSM/people/mlicanqueo) (master)
* [Sebastián Toro](https://github.com/orgs/POWERLAB-UTFSM/people/sebatorod) (master)
* [José Figueroa](https://github.com/orgs/POWERLAB-UTFSM/people/joafce) (master)

## Sub-Carpetas

## Software para modificar repositorio
* [Code Composer Studio](https://www.ti.com/tool/CCSTUDIO)
* [ControlSUITE](https://www.ti.com/tool/CONTROLSUITE)
* [C2000Ware](https://www.ti.com/tool/C2000WARE)

## Detalles técnicos del diseño
| **Descripción** | **Tipo** |
| - | - |
| Convertidor | AC-DC | 
| Topología | Boost trifásico | 
| Aislación Galvánica | No | 
| Potencia nominal | 5000 W | 
| Tensión de entrada nominal | 380 V línea-línea | 
| Tensión de salida nominal | 750 VDC | 
| Semiconductores de potencia | SiC MOSFET | 
| Control | Microcontrolador embebido (c2000 - Texas Instruments) | 

## Usar Git con este proyecto
### Descargar (clonar) este proyecto al computador local:
1. Instalar [Git](https://git-scm.com/downloads).
1. Instalar [Git LFS](https://git-lfs.github.com/).
1. Abrir la consola de comandos de Git e inicializar Git LFS:<br />
`git lfs install`
1. (*Opcional*) Instalar [Git Extensions](http://gitextensions.github.io), [GitHub Desktop](https://desktop.github.com/), o algún otro GUI para Git.
1. Ir al directorio principal donde se quiere dejar el repositorio local, y clonar desde el repositorio remoto `https://remote.git`:<br />
`git clone https://remote.git` <br />
En este caso: <br />
`git clone https://github.com/POWERLAB-UTFSM/Control_AC3DC_Boost2L_F2837xD`

### Usar Git desde la línea de comandos (manual)
1. **Abrir Bash:** <br /> Una vez que se hayan hecho las actualizaciones, ve al directorio del proyecto, abre la ventana de comandos de Git (Git Bash, u otro) y ejecuta:
1. `git status` <br /> Muestra los cambios hechos.
1. `git add [path/file]` <br /> Prepara los archivos/carpetas efectivos para consolidar (**Commit**). Para añadir todos los archivos a la vez, ejecutar `git add .`.
1. `git commit -m "description"` <br /> Consolida los cambios hechos con su respectiva descripción, y agrega el comentario `description`.
1. `git push origin master` <br /> Sube los cambios desde el repositorio local `origin` y la rama `master`, al repositorio remoto. Cambiar el nombre del repositorio local y rama, de ser necesario.
1. `git pull origin master` <br /> Descarga los últimos cambios desde el repositorio remoto hacia el repositorio local `origin` y la rama `master`.
1. `git checkout <branch>` <br /> Cambia a la rama `<branch>`.

Para más información: https://git-scm.com/doc

### Usar Git con Code Composer Studio
http://software-dl.ti.com/ccs/esd/documents/sdto_ccs_git-with-ccs.html

## Otros comentarios
:D
