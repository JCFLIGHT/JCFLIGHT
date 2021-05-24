import optparse
import yaml
import codecs

SETTINGS_YAML_PATH = "Settings.yaml"
SETTINGS_MD_PATH = "Settings.md"


def Parse_Settings_YAML():
    with codecs.open(SETTINGS_YAML_PATH, "r", "utf-8-sig") as Settings_YAML:
        return yaml.load(Settings_YAML, Loader=yaml.Loader)


def Generate_MD_Doc_From_YAML(Settings_YAML):
    Params = {}

    for Member in Settings_YAML['Members']:

        Params[Member['Name']] = {
            "Descrição": Member["Description"] if "Description" in Member else "",
            "Valor Padrão": Member["DefaultValue"] if "DefaultValue" in Member else "",
            "Min": Member["Min"] if "Min" in Member else "",
            "Max": Member["Max"] if "Max" in Member else ""
        }

    MD_Table = [
        "| Nome | Valor Padrão | Min | Max | Descrição |\n",
        "| ------------- | ------------- | --- | --- | ----------- |\n",
    ]

    for Param in sorted(Params.items()):
        MD_Table.append("| {} | {} | {} | {} | {} |\n".format(
            Param[0], Param[1]['Valor Padrão'], Param[1]['Min'], Param[1]['Max'], Param[1]['Descrição']
        ))

    return MD_Table


def Write_Settings_MD(Lines):

    with codecs.open(SETTINGS_MD_PATH, "w", "utf-8-sig") as Settings_md:
        Settings_md.writelines(Lines)


if __name__ == "__main__":
    Settings_YAML = Parse_Settings_YAML()
    MD_Table = Generate_MD_Doc_From_YAML(Settings_YAML)
    Write_Settings_MD(["# Variáveis do CLI\n", "\n"] + MD_Table + ["\n",
                      "> Esse arquivo é gerado automaticamente,não o edite manualmente!"])
