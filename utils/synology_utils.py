import os
import requests
from tqdm import tqdm


class SynologySession:
    def __init__(
        self,
        address: str,
        port: int,
        username: str,
        password: str,
        session: str,
        max_retry: int = 2,
    ) -> None:
        self._password = password
        self._sid = ""

        self.base_address = f"{address}:{port}"
        self.username = username
        self.session = session
        self.max_retry = max_retry
        self.api_info = self.get_api_info()

    def __enter__(self):
        self._sid = self._login()
        print(f"Connection to {self.base_address} is opened: {self._sid}")
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        logout_response = self._logout()
        print(f"Connection to {self.base_address} is closed: {logout_response}")

    def _login(self) -> str:
        api = "SYNO.API.Auth"
        login_response = self._request_get_with_retries(
            api_path="webapi/auth.cgi",
            api=api,
            version=self.api_info[api]["maxVersion"],
            method="login",
            account=self.username,
            passwd=self._password,
            session=self.session,
            format="sid",
        )
        if login_response.json()["success"]:
            return login_response.json()["data"]["sid"]
        else:
            return ""

    def _logout(self) -> bool:
        api = "SYNO.API.Auth"
        logout_response = self._request_get_with_retries(
            api_path="webapi/auth.cgi",
            api=api,
            version=self.api_info[api]["maxVersion"],
            method="logout",
            session=self.session,
        )
        return logout_response.json()["success"]

    def _request_get_with_retries(
        self,
        api_path: str,
        api: str,
        version: int,
        method: str,
        account: str = None,
        passwd: str = None,
        query: str = None,
        session: str = None,
        format: str = None,
        path: list = [],
        folder_path: list = [],
        name: list = [],
        mode: str = None,
        force_parent: bool = False,
        is_sid: bool = False,
    ) -> requests.Response:
        retry_count = 0

        while True:
            retry_count += 1
            request_url = self._get_request_url(
                api_path=api_path,
                api=api,
                version=version,
                method=method,
                account=account,
                passwd=passwd,
                query=query,
                session=session,
                format=format,
                path=path,
                folder_path=folder_path,
                name=name,
                mode=mode,
                force_parent=force_parent,
                is_sid=is_sid,
            )
            response = requests.get(request_url)

            if response.status_code == 200:
                return response

            if retry_count >= self.max_retry:
                return response

    def _get_request_url(
        self,
        api_path: str,
        api: str,
        version: int,
        method: str,
        account: str = None,
        passwd: str = None,
        query: str = None,
        session: str = None,
        format: str = None,
        path: list = [],
        folder_path: list = [],
        name: list = [],
        mode: str = None,
        force_parent: bool = False,
        is_sid: bool = False,
    ) -> requests.Response:
        request_url = f"{self.base_address}/{api_path}?api={api}&version={version}&method={method}"
        request_url += f"&account={account}" if account is not None else ""
        request_url += f"&passwd={passwd}" if passwd is not None else ""
        request_url += f"&query={query}" if query is not None else ""
        request_url += f"&session={session}" if session is not None else ""
        request_url += f"&format={format}" if format is not None else ""
        request_url += (
            f"&path={path}".replace(" ", "").replace("'", '"') if len(path) != 0 else ""
        )
        request_url += (
            f"&folder_path={folder_path}".replace(" ", "").replace("'", '"')
            if len(folder_path) != 0
            else ""
        )
        request_url += (
            f"&name={name}".replace(" ", "").replace("'", '"') if len(name) != 0 else ""
        )
        request_url += f"&mode={mode}" if mode is not None else ""
        # request_url += f"&force_parent={force_parent}".lower() if force_parent else ""
        request_url += f"&_sid={self._sid}" if is_sid else ""
        # print(request_url)
        return request_url

    def get_api_info(self) -> dict:
        api_info_response = self._request_get_with_retries(
            api_path="webapi/query.cgi",
            api="SYNO.API.Info",
            version=1,
            method="query",
            query="SYNO.API.Info,SYNO.API.Auth,SYNO.FileStation.Info,SYNO.FileStation.Upload,SYNO.FileStation.Download,SYNO.FileStation.CreateFolder",
        )
        if api_info_response.json()["success"]:
            return api_info_response.json()["data"]
        else:
            return api_info_response.json()["error"]

    def get_filestation_info(self) -> dict:
        api = "SYNO.FileStation.Info"
        api_info_response = self._request_get_with_retries(
            api_path="webapi/entry.cgi",
            api=api,
            version=self.api_info[api]["maxVersion"],
            method="get",
            is_sid=True,
        )
        if api_info_response.json()["success"]:
            return api_info_response.json()["data"]
        else:
            return api_info_response.json()["error"]

    def create_folder(self, folder_path: list, name: list, force_parent: bool) -> dict:
        api = "SYNO.FileStation.CreateFolder"
        create_folder_response = self._request_get_with_retries(
            api_path="webapi/entry.cgi",
            api=api,
            version=self.api_info[api]["maxVersion"],
            method="create",
            folder_path=folder_path,
            name=name,
            force_parent=force_parent,
            is_sid=True,
        )
        if create_folder_response.json()["success"]:
            return create_folder_response.json()["data"]
        else:
            return create_folder_response.json()["error"]

    def download_folder_file(
        self, path: list, save_folder_path: str, is_dir: bool = False
    ) -> str:
        api = "SYNO.FileStation.Download"
        request_url = self._get_request_url(
            api_path="webapi/entry.cgi",
            api=api,
            version=self.api_info[api]["maxVersion"],
            method="download",
            path=path,
            mode="download",
            is_sid=True,
        )

        session = requests.session()
        with session.get(request_url, stream=True) as r:
            r.raise_for_status()
            if is_dir:
                file_name = os.path.basename(path[0]) + ".zip"
            else:
                file_name = os.path.basename(path[0]) if len(path) == 1 else "temp.zip"
            file_path = os.path.join(save_folder_path, file_name)
            with open(file_path, "wb") as f:
                pbar = tqdm(r.iter_content(chunk_size=8192))
                for idx, chunk in enumerate(pbar):
                    pbar.set_description(f"{idx/128.0:.2f} MB")
                    if chunk:  # filter out keep-alive new chunks
                        f.write(chunk)

        return file_path

    def upload_files(
        self,
        path: str,
        upload_folder: str,
        file_path_list: list,
        create_parents: bool = False,
        overwrite: str = "skip",
    ) -> list:
        api = "SYNO.FileStation.Upload"
        request_url = self._get_request_url(
            api_path="webapi/entry.cgi",
            api=api,
            version=self.api_info[api]["maxVersion"],
            method="upload",
            is_sid=True,
        )

        session = requests.session()
        result = []
        pbar = tqdm(file_path_list)
        for file_path in pbar:
            pbar.set_description(f"[{file_path[len(upload_folder):]}]")
            with open(file_path, "rb") as payload:
                args = {
                    "path": path,
                    "create_parents": str(create_parents).lower(),
                    "overwrite": str(overwrite).lower(),
                }
                files = {
                    "file": (
                        os.path.basename(file_path),
                        payload,
                        "application/octet-stream",
                    )
                }
                r = session.post(request_url, data=args, files=files)
                if r.status_code == 200 and r.json()["success"]:
                    result.append(f"{file_path} Upload Complete")
                else:
                    result.append(f"{file_path} Upload Fail: {r.status_code} {r.json()}")
        return result
